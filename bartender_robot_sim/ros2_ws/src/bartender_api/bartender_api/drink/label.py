"""Which bottle stands at a station: OCR reads its label, Gemini is the fallback.

PaddleOCR reads the label text locally, and regexes pull the type, volume and
ABV out of it; the brand can only come from the inventory
(config/bottles.yaml), since no pattern says which word is a brand. Gemini
runs at the same time; its answer is used only when OCR did not name a
stocked bottle, and OCR's partial answer stands when Gemini is off or cannot
help.

Any reading that names an inventory row is replaced by that row: both readers
misread stylised or Japanese brand names, and a volume read can change between
runs. A bottle not in the inventory is reported as read, at a lower
confidence, but only from evidence in the text read: a brand Gemini names must
appear in its text, and a type must be spelled on the label, so a guess from
the bottle's shape is not reported. `read` always keeps the raw reading.
"""
import concurrent.futures
import dataclasses
import difflib
import re
import unicodedata
from dataclasses import dataclass

import yaml

# ponytail: calibration knob; brand similarity (0..1) needed to accept a row.
MIN_BRAND_MATCH = 0.8
# ponytail: calibration knob; fuzzy() refuses when the top two brands are closer than this.
FUZZY_MARGIN = 0.1
# Reported when the brand was not read and only one row has the type read.
TYPE_ONLY_CONFIDENCE = 0.6
# Reported for a bottle not in the inventory, brand and type as Gemini read them.
READ_CONFIDENCE = 0.5
# Reported when only the type spelled on the label is known.
TYPE_READ_CONFIDENCE = 0.4

# Words that name each type on a label, matched on accent-folded, casefolded text.
# Order matters: the first type whose pattern matches wins.
TYPE_WORDS = {
    'whiskey': r'whiske?y|bourbon|scotch',
    'gin': r'\bgin\b|ジン',
    'vodka': r'vodka|wodka',
    'rum': r'\brum\b|\bron\b|\brhum\b',
    'tequila': r'tequila|mezcal',
    'brandy': r'brandy|cognac|armagnac',
    'liqueur': r'liqueur|likor|liquore',
    'wine': r'\bwine\b|\bvin\b|\bwein\b',
    'beer': r'\bbeer\b|\bbier\b|\bpiwo\b|\blager\b',
}
# OCR drops letters ('40% VoL. 70c'), so a bare 'c' counts as cl.
VOLUME = re.compile(r'(\d+(?:[.,]\d+)?) ?(ml|cl|c|ltr|litre|liter|l)\b')
ML_PER = {'ml': 1, 'cl': 10, 'c': 10, 'l': 1000, 'ltr': 1000, 'litre': 1000,
          'liter': 1000}
# '%' must be followed by vol/alc, so OCR noise like '4%0' is not an ABV.
ABV = re.compile(r'(\d+(?:[.,]\d+)?) ?% ?(?:v|alc|abv)')

# ponytail: two workers, so a Gemini call left running after OCR answered
# delays the next read's call; more workers if several bottles arrive at once.
_gemini_pool = concurrent.futures.ThreadPoolExecutor(
    max_workers=2, thread_name_prefix='gemini')

PROMPT = (
    'This is a camera crop of one liquor bottle on a bartending robot. '
    'Report only what is printed on its label: brand, the kind of alcohol, '
    'volume in ml and ABV %. Use "" or 0 for anything you cannot read; do '
    'not guess. text_read is the label text you could actually read.')


@dataclass
class Bottle:
    """One inventory row: a bottle the bar actually stocks."""
    brand: str
    type: str
    volume_ml: int
    aliases: tuple = ()


@dataclass
class Label:
    """What a station's bottle is. `type` None means don't know, never no bottle."""
    brand: str | None
    type: str | None
    volume_ml: int | None
    confidence: float
    source: str | None
    reason: str | None = None
    read: dict | None = None


def unknown(reason, read=None):
    return Label(None, None, None, 0.0, None, reason, read)


def load_inventory(path):
    with open(path, encoding='utf-8') as f:
        rows = yaml.safe_load(f)['bottles']
    return [Bottle(r['brand'], r['type'], int(r['volume_ml']),
                   tuple(r.get('aliases', ()))) for r in rows]


def schema(inventory):
    """Gemini's answer shape; type is one of TYPE_WORDS or the inventory's types."""
    types = sorted(set(TYPE_WORDS) | {b.type for b in inventory}) + ['other', 'unknown']
    return {
        'type': 'object',
        'properties': {
            'brand': {'type': 'string'},
            'type': {'type': 'string', 'enum': types},
            'volume_ml': {'type': 'integer'},
            'abv_percent': {'type': 'number'},
            'text_read': {'type': 'string'},
        },
        'required': ['brand', 'type', 'volume_ml', 'abv_percent', 'text_read'],
    }


def _fold(text):
    """Casefold and drop accents: 'WÓDKA' -> 'wodka'."""
    text = unicodedata.normalize('NFKD', text.casefold())
    return ''.join(c for c in text if not unicodedata.combining(c))


def _norm(text):
    """_fold, then drop spaces and punctuation: "Jack Daniel's" -> 'jackdaniels'."""
    return ''.join(c for c in _fold(text) if c.isalnum())


def _type_in_text(text):
    folded = _fold(text)
    return next((t for t, p in TYPE_WORDS.items() if re.search(p, folded)), None)


def parse(text):
    """Turn OCR'd label text into a reading shaped like Gemini's; the inventory finds the brand."""
    folded = _fold(text)
    volume = VOLUME.search(folded)
    abv = ABV.search(folded)
    ml = round(float(volume[1].replace(',', '.')) * ML_PER[volume[2]]) if volume else 0
    percent = float(abv[1].replace(',', '.')) if abv else 0.0
    return {'brand': '', 'type': _type_in_text(text) or 'unknown',
            'volume_ml': ml if 20 <= ml <= 5000 else 0,
            'abv_percent': percent if 0 < percent < 100 else 0.0,
            'text_read': text}


def _brand_score(read, bottle):
    names = [n for n in map(_norm, (bottle.brand, *bottle.aliases)) if n]
    text = _norm(read.get('text_read', ''))
    if any(n in text for n in names):
        return 1.0
    brand = _norm(read.get('brand', ''))
    if not brand:
        return 0.0
    return max(difflib.SequenceMatcher(None, brand, n).ratio() for n in names)


def fuzzy(text, options):
    """Pick the option whose spelling best matches a run of words in `text`.

    Jev-style typed choice -- options {name: [spellings]} in, (name | None,
    confidence) out -- but rule-based: difflib against every run of up to one
    word more than the spelling, since OCR splits and merges words. None when
    nothing reaches MIN_BRAND_MATCH or the top two are within FUZZY_MARGIN.
    A model backend (Jev, Laya) can stand in with the same signature.
    """
    words = [w for w in map(_norm, _fold(text).split()) if w]
    runs = {''.join(words[i:i + n]) for n in range(1, 5)
            for i in range(len(words) - n + 1)}

    def score(spelling):
        target = _norm(spelling)
        return max((difflib.SequenceMatcher(None, target, r).ratio()
                    for r in runs if target), default=0.0)
    ranked = sorted(((max(map(score, spellings), default=0.0), name)
                     for name, spellings in options.items()), reverse=True)
    if not ranked or ranked[0][0] < MIN_BRAND_MATCH:
        return None, 0.0
    if len(ranked) > 1 and ranked[0][0] - ranked[1][0] < FUZZY_MARGIN:
        return None, 0.0
    return ranked[0][1], round(ranked[0][0], 3)


def _readable_new_bottle(read):
    brand = _norm(read.get('brand', ''))
    return (read.get('type') not in ('other', 'unknown', None) and bool(brand)
            and brand in _norm(read.get('text_read', '')))


def match(read, inventory, source):
    """Pick the inventory row a reading names, else what it read, else unknown with why."""
    # Brand first; among equal brands, the row whose volume was read.
    score, _, best = max(
        ((_brand_score(read, b), b.volume_ml == read.get('volume_ml'), b)
         for b in inventory), key=lambda s: s[:2], default=(0.0, False, None))
    if score >= MIN_BRAND_MATCH:
        return Label(best.brand, best.type, best.volume_ml, round(score, 3),
                     'inventory', read=read)
    volume = read.get('volume_ml') or None
    if _readable_new_bottle(read):
        return Label(read['brand'], read['type'], volume, READ_CONFIDENCE, source,
                     'not in the inventory; as read from the label', read)
    seen = _type_in_text(read.get('text_read', ''))
    same_type = [b for b in inventory if b.type == seen]
    if seen and len(same_type) == 1:
        b = same_type[0]
        return Label(b.brand, b.type, b.volume_ml, TYPE_ONLY_CONFIDENCE,
                     'inventory', f'brand not read; the only {b.type} in the '
                     'inventory', read)
    if seen:
        return Label(None, seen, volume, TYPE_READ_CONFIDENCE, source,
                     'brand not read; type as spelled on the label', read)
    return unknown(f'{source} read no stocked brand, and no type on the label',
                   read)


def _decided(text, inventory, decide):
    """The inventory row `decide` picks from OCR text, as a Label, or None."""
    brand, confidence = decide(
        text, {b.brand: [b.brand, *b.aliases] for b in inventory})
    if brand is None:
        return None
    # match() still breaks a tie between sizes of the chosen brand by the volume read.
    row = match(dict(parse(text), brand=brand), inventory, 'ocr')
    return dataclasses.replace(row, confidence=confidence, source=decide.__name__,
                               reason=f'brand picked from the OCR text by {decide.__name__}')


def _knows(reading):
    # Anything but 'inventory' here came from _decided: gemini and ocr rows are never `known`.
    return reading.source not in (None, 'ocr', 'gemini') and reading.confidence >= MIN_BRAND_MATCH


def _read_locally(crop, inventory, ocr, deciders):
    """OCR's reading, or when it names no stocked brand, the first sure pick of deciders."""
    if ocr is None:
        return unknown('no OCR (pip install paddlepaddle paddleocr)')
    text, why = ocr(crop)
    if text is None:
        return unknown(why)
    local = match(parse(text), inventory, 'ocr')
    if _knows(local):
        return local
    for decide in deciders:
        picked = _decided(text, inventory, decide)
        if picked is not None and _knows(picked):
            return picked
    return local


def read_label(crop, inventory, ocr=None, ask=None, deciders=(fuzzy,)):
    """Run OCR and Gemini at once; OCR's answer if it names a stocked bottle, else Gemini's.

    `deciders` get a second look, in order, at OCR text that names no
    stocked brand outright, before waiting on Gemini: the fast local ones
    first, a remote model (jev) as the fallback. () skips it.
    """
    # Costs a Gemini call even when OCR then knows the bottle, for no wait when it doesn't.
    remote_job = _gemini_pool.submit(ask, crop) if ask is not None else None
    local = _read_locally(crop, inventory, ocr, deciders)
    if _knows(local):
        if remote_job is not None:
            remote_job.cancel()   # only stops a call not yet started
        return local
    if remote_job is None:
        return local
    answer, why = remote_job.result()
    remote = unknown(why) if answer is None else match(answer, inventory, 'gemini')
    if remote.type is not None:
        return remote
    if local.type is not None:
        return local
    return unknown(f'{local.reason}; {remote.reason}', remote.read or local.read)
