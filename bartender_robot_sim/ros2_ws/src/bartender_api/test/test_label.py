"""Label reading -> inventory row, from what PaddleOCR and Gemini read in phone photos."""
import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api.drink import label                       # noqa: E402

INVENTORY = label.load_inventory(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'config',
    'bottles.yaml'))

# PaddleOCR on bottle crops from phone photos, at ocr.SCALE; Żubrówka's label read as noise.
OCR_TEXT = {
    'jd_949': "MF IOK DSWIELS JACKDANIEL'S 26N Tennessee WHISKEY Jara 40% VoL. 70c",
    'jd_952': "MF PACK DANIES JACK DANIEL'S 207 BRAND Tennessee WHISKEY 40% VhL. 70CL",
    'zub_949': '20 BRO MF 鞋 AJA18 50',
    'gin_949': '天雀 GIN',
    'gin_952': '5 GIN',
}


def _read(brand='', type_='unknown', volume_ml=0, text_read=''):
    return {'brand': brand, 'type': type_, 'volume_ml': volume_ml,
            'abv_percent': 40.0, 'text_read': text_read}


def _ocr(text):
    return lambda crop: (text, None)


def _gemini(answer):
    asked = []

    def ask(crop):
        asked.append(crop)
        return answer
    return ask, asked


@pytest.mark.parametrize('text, expected', [
    (OCR_TEXT['jd_949'], ('whiskey', 700, 40.0)),
    (OCR_TEXT['jd_952'], ('whiskey', 700, 40.0)),
    (OCR_TEXT['zub_949'], ('unknown', 0, 0.0)),
    (OCR_TEXT['gin_952'], ('gin', 0, 0.0)),
    ('BIAŁA POLSKA WÓDKA 40% vol 500 ml', ('vodka', 500, 40.0)),
    ('Tequila 38 % alc 0,75 l', ('tequila', 750, 38.0)),
    ('4%0 GINGER', ('unknown', 0, 0.0)),
])
def test_parse_pulls_type_volume_and_abv_from_ocr_text(text, expected):
    got = label.parse(text)
    assert (got['type'], got['volume_ml'], got['abv_percent']) == expected


@pytest.mark.parametrize('text, brand', [
    (OCR_TEXT['jd_949'], "Jack Daniel's"),
    (OCR_TEXT['jd_952'], "Jack Daniel's"),
    (OCR_TEXT['gin_949'], 'Tenjaku'),
])
def test_ocr_naming_a_stocked_brand_wins_over_gemini(text, brand):
    ask, _ = _gemini((_read('Patron', 'tequila', 750, 'PATRON TEQUILA'), None))
    got = label.read_label('crop', INVENTORY, _ocr(text), ask)
    assert (got.brand, got.source, got.confidence) == (brand, 'inventory', 1.0)


def test_ocr_and_gemini_run_at_the_same_time():
    gemini_started = threading.Event()

    def ocr(crop):
        # Only returns if Gemini was started while OCR was still reading.
        assert gemini_started.wait(5), 'Gemini did not start alongside OCR'
        return OCR_TEXT['gin_952'], None

    def ask(crop):
        gemini_started.set()
        return _read("Hendrick's", 'gin', 700, "HENDRICK'S GIN"), None
    assert label.read_label('crop', INVENTORY, ocr, ask).brand == "Hendrick's"


def test_ocr_answer_does_not_wait_for_a_slow_gemini():
    release = threading.Event()

    def slow_ask(crop):
        release.wait(5)
        return None, 'gemini: late'
    start = time.monotonic()
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['jd_949']), slow_ask)
    elapsed = time.monotonic() - start
    release.set()
    assert got.brand == "Jack Daniel's"
    assert elapsed < 1.0


def test_ocr_type_only_asks_gemini_and_gemini_wins():
    ask, asked = _gemini((_read("Hendrick's", 'gin', 700, "HENDRICK'S GIN"), None))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['gin_952']), ask)
    assert (got.brand, got.source) == ("Hendrick's", 'gemini')
    assert asked == ['crop']


def test_gemini_down_keeps_ocrs_partial_answer():
    ask, _ = _gemini((None, 'gemini: 429 quota'))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['gin_952']), ask)
    assert (got.brand, got.confidence) == ('Tenjaku', label.TYPE_ONLY_CONFIDENCE)


def test_offline_new_bottle_gets_its_type_but_no_brand():
    got = label.read_label('crop', INVENTORY, _ocr('BACARDI CARTA BLANCA RUM 70cl'))
    assert (got.brand, got.type, got.volume_ml) == (None, 'rum', 700)
    assert (got.source, got.confidence) == ('ocr', label.TYPE_READ_CONFIDENCE)


def test_nothing_read_anywhere_is_unknown_with_both_reasons():
    ask, _ = _gemini((None, 'gemini: 503'))
    got = label.read_label('crop', INVENTORY, _ocr(OCR_TEXT['zub_949']), ask)
    assert got.type is None
    assert 'ocr read no stocked brand' in got.reason
    assert 'gemini: 503' in got.reason


def test_gemini_alone_works_without_ocr():
    ask, _ = _gemini((_read('ŻUBRÓWKA', 'vodka', 0, 'ŻUBRÓWKA VODKA BISON GRASS'), None))
    got = label.read_label('crop', INVENTORY, None, ask)
    assert (got.brand, got.volume_ml) == ('Żubrówka', 500)


def test_ocr_failure_is_reported():
    got = label.read_label('crop', INVENTORY, lambda crop: (None, 'ocr: boom'))
    assert got.type is None
    assert got.reason == 'ocr: boom'


def test_no_reader_at_all_says_so():
    assert 'no OCR' in label.read_label('crop', INVENTORY).reason


def test_accents_and_case_do_not_matter():
    got = label.match(_read('ZUBROWKA', 'vodka', 500, 'BIALA POLSKA WODKA'),
                      INVENTORY, 'gemini')
    assert got.brand == 'Żubrówka'


def test_type_and_volume_come_from_the_inventory_not_the_reading():
    got = label.match(_read("Jack Daniel's", 'other', 1000), INVENTORY, 'gemini')
    assert (got.type, got.volume_ml) == ('whiskey', 700)
    assert got.read['volume_ml'] == 1000


def test_volume_read_breaks_a_tie_between_sizes_of_one_brand():
    stock = INVENTORY + [label.Bottle("Jack Daniel's", 'whiskey', 1000)]
    assert label.match(_read("Jack Daniel's", 'whiskey', 1000), stock,
                       'gemini').volume_ml == 1000
    assert label.match(_read("Jack Daniel's", 'whiskey', 700), stock,
                       'gemini').volume_ml == 700


def test_new_bottle_is_reported_as_gemini_read_it():
    got = label.match(_read("Hendrick's", 'gin', 700, "HENDRICK'S GIN 41.4% 70cl"),
                      INVENTORY, 'gemini')
    assert (got.brand, got.type, got.volume_ml) == ("Hendrick's", 'gin', 700)
    assert (got.source, got.confidence) == ('gemini', label.READ_CONFIDENCE)


def test_brand_or_type_absent_from_the_text_is_a_guess_and_not_reported():
    # From above, Gemini named the Jack Daniel's bottle Patron with no label text.
    got = label.match(_read('Patron', 'tequila', 750, ''), INVENTORY, 'gemini')
    assert got.type is None
    assert got.read['brand'] == 'Patron'


def test_type_spelled_on_the_label_of_an_ambiguous_type_has_no_brand():
    stock = INVENTORY + [label.Bottle('Absolut', 'vodka', 700)]
    got = label.match(_read('', 'vodka', 0, 'VODKA 40%'), stock, 'gemini')
    assert (got.brand, got.type) == (None, 'vodka')


def test_empty_inventory_still_reports_what_gemini_read():
    got = label.match(_read("Jack Daniel's", 'whiskey', 700, "JACK DANIEL'S"), [],
                      'gemini')
    assert (got.brand, got.source) == ("Jack Daniel's", 'gemini')


def test_schema_offers_general_and_inventory_types():
    stock = INVENTORY + [label.Bottle('Soju Co', 'soju', 360)]
    enum = label.schema(stock)['properties']['type']['enum']
    assert {'rum', 'tequila', 'whiskey', 'soju'} <= set(enum)
    assert enum[-2:] == ['other', 'unknown']


BRANDS = {b.brand: [b.brand, *b.aliases] for b in INVENTORY}


@pytest.mark.parametrize('text, brand', [
    ("OLD NO.7 JACK DANIELS TENNESSEE", "Jack Daniel's"),
    ("JACK DANIEL'S", "Jack Daniel's"),
    ("JAK DANIELS WHISKEY", "Jack Daniel's"),
    ("ZUBROWKA BISON GRASS", 'Żubrówka'),
    ("TENJAK GIN", 'Tenjaku'),
])
def test_fuzzy_finds_a_misspelt_or_split_brand(text, brand):
    assert label.fuzzy(text, BRANDS)[0] == brand


@pytest.mark.parametrize('text', [
    'BACARDI CARTA BLANCA RUM 70cl', OCR_TEXT['zub_949'], OCR_TEXT['gin_952'], ''])
def test_fuzzy_names_nothing_for_an_unrelated_or_noisy_label(text):
    assert label.fuzzy(text, BRANDS) == (None, 0.0)


def test_fuzzy_refuses_two_brands_too_close_to_call():
    assert label.fuzzy('GORDONS', {"Gordon's": ["Gordon's"], 'Gordon': ['Gordon']}) == (None, 0.0)


def test_fuzzy_hit_comes_from_the_inventory_without_waiting_on_gemini():
    release = threading.Event()

    def slow_ask(crop):
        release.wait(5)
        return None, 'gemini: late'
    start = time.monotonic()
    got = label.read_label('crop', INVENTORY, _ocr('JAK DANIELS WHISKEY 70cl'), slow_ask)
    elapsed = time.monotonic() - start
    release.set()
    assert (got.brand, got.type, got.volume_ml, got.source) == (
        "Jack Daniel's", 'whiskey', 700, 'fuzzy')
    assert label.MIN_BRAND_MATCH <= got.confidence < 1.0
    assert elapsed < 1.0


def test_no_deciders_leaves_the_old_path():
    ask, asked = _gemini((None, 'gemini: 503'))
    got = label.read_label('crop', INVENTORY, _ocr('JAK DANIELS WHISKEY'), ask, deciders=())
    assert got.source != 'fuzzy'
    assert asked == ['crop']


def test_decide_is_swappable_for_a_model_backend():
    def model(text, options):
        assert "Tenjaku" in options
        return 'Tenjaku', 0.93
    got = label.read_label('crop', INVENTORY, _ocr('a bird on a japanese gin'), deciders=(model,))
    assert (got.brand, got.confidence, got.source) == ('Tenjaku', 0.93, 'model')


def _model(pick, calls):
    def jev(text, options):
        calls.append(text)
        return pick
    return jev


def test_jev_is_asked_only_when_fuzzy_finds_nothing():
    calls = []
    got = label.read_label('crop', INVENTORY, _ocr('JAK DANIELS WHISKEY'),
                           deciders=(label.fuzzy, _model(('Tenjaku', 0.99), calls)))
    assert (got.brand, got.source) == ("Jack Daniel's", 'fuzzy')
    assert calls == []


def test_jev_picks_when_fuzzy_finds_nothing():
    calls = []
    got = label.read_label('crop', INVENTORY, _ocr('a bird on a japanese gin'),
                           deciders=(label.fuzzy, _model(('Tenjaku', 0.9), calls)))
    assert (got.brand, got.source, got.confidence) == ('Tenjaku', 'jev', 0.9)
    assert calls == ['a bird on a japanese gin']


def test_an_unsure_jev_pick_is_not_taken():
    ask, asked = _gemini((None, 'gemini: 503'))
    got = label.read_label('crop', INVENTORY, _ocr('a bird on a japanese gin'), ask,
                           deciders=(label.fuzzy, _model(('Tenjaku', 0.5), [])))
    assert got.source != 'jev'
    assert asked == ['crop']


# A wider bar than bottles.yaml: one of each other kind, and two rums so a
# type alone cannot name the brand.
WIDE_BAR = [
    label.Bottle('Bacardi', 'rum', 700),
    label.Bottle('Captain Morgan', 'rum', 700),
    label.Bottle('Jose Cuervo', 'tequila', 700),
    label.Bottle('Heineken', 'beer', 330),
    label.Bottle('Jägermeister', 'liqueur', 700, ('Jagermeister',)),
    label.Bottle('Hennessy', 'brandy', 700),
    label.Bottle('Yellow Tail', 'wine', 750),
]


@pytest.mark.parametrize('text, brand, type_, volume', [
    ('BACARDI CARTA BLANCA SUPERIOR RUM 70cl', 'Bacardi', 'rum', 700),
    ('CAPTAIN MORGAN ORIGINAL SPICED 35% vol 70cl', 'Captain Morgan', 'rum', 700),
    ('JOSE CUERVO ESPECIAL TEQUILA 38% alc 0,7 l', 'Jose Cuervo', 'tequila', 700),
    ('Heineken LAGER BEER 5% vol 330 ml', 'Heineken', 'beer', 330),
    ('JÄGERMEISTER KRÄUTERLIKÖR 35% vol', 'Jägermeister', 'liqueur', 700),
    ('HENNESSY V.S COGNAC 40% vol', 'Hennessy', 'brandy', 700),
    ('[yellow tail] SHIRAZ WINE OF AUSTRALIA 750ml', 'Yellow Tail', 'wine', 750),
])
def test_other_kinds_of_bottle_are_named_from_the_inventory(text, brand, type_, volume):
    got = label.read_label('crop', WIDE_BAR, _ocr(text))
    assert (got.brand, got.type, got.volume_ml) == (brand, type_, volume)


@pytest.mark.parametrize('text, brand', [
    ('BACARD1 SUPERIOR', 'Bacardi'),
    ('HEINEKN LAGER', 'Heineken'),
    ('JAGERMEISTR', 'Jägermeister'),
    ('HENESSY COGNAC', 'Hennessy'),
    ('JOSE CUERV0', 'Jose Cuervo'),
])
def test_fuzzy_names_other_kinds_misread_by_ocr(text, brand):
    got = label.read_label('crop', WIDE_BAR, _ocr(text))
    assert (got.brand, got.source) == (brand, 'fuzzy')


@pytest.mark.parametrize('text, type_', [
    ('Tequila 38 % alc 0,75 l', 'tequila'),
    ('COGNAC FINE CHAMPAGNE', 'brandy'),
    ('ROTWEIN wein 0,75 l', 'wine'),
    ('PREMIUM LIQUEUR', 'liqueur'),
    ('CRAFT BIER', 'beer'),
])
def test_parse_reads_the_type_of_other_kinds(text, type_):
    assert label.parse(text)['type'] == type_


def test_two_rums_and_no_brand_is_type_only_without_a_brand():
    got = label.read_label('crop', WIDE_BAR, _ocr('DARK RUM 70cl'))
    assert (got.brand, got.type, got.volume_ml) == (None, 'rum', 700)
    assert got.confidence == label.TYPE_READ_CONFIDENCE


def test_one_tequila_and_no_brand_is_that_tequila():
    got = label.read_label('crop', WIDE_BAR, _ocr('100% AGAVE TEQUILA'))
    assert (got.brand, got.confidence) == ('Jose Cuervo', label.TYPE_ONLY_CONFIDENCE)
