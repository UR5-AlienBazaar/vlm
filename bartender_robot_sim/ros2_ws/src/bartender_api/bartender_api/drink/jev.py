"""Ask TypeSafe's Jev typed questions through OpenRouter's decisions API.

Off unless JEV_KEY (an OpenRouter API key; OPENROUTER_API_KEY also works) is
set. Every call sends its state text to OpenRouter. make_jev is label.fuzzy's
fallback, asked only when fuzzy finds no brand; intent.py routes /ask through
make_asker.
"""
import json
import os
import sys
import urllib.error
import urllib.request

URL = 'https://openrouter.ai/api/alpha/decisions'
MODEL = 'typesafe/jev-1.13'
# ponytail: calibration knob; Jev answers in 70-500 ms, so a slow call is a stuck one.
TIMEOUT_S = 2.0
NONE = 'none'


def make_asker():
    """Return ask(state, questions) -> (answers, None) or (None, why); None if not configured."""
    key = os.environ.get('JEV_KEY') or os.environ.get('OPENROUTER_API_KEY')
    if not key:
        return None

    def ask(state, questions):
        body = {'model': MODEL, 'state': state, 'questions': questions}
        request = urllib.request.Request(
            URL, data=json.dumps(body).encode(), method='POST',
            headers={'Authorization': f'Bearer {key}',
                     'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
                return json.load(response)['answers'], None
        except (urllib.error.URLError, TimeoutError, KeyError, ValueError) as exc:
            return None, f'jev: {exc}'

    return ask


def make_jev():
    """Return jev(text, options) -> (brand | None, confidence), or None when not configured."""
    ask = make_asker()
    if ask is None:
        return None

    def jev(text, options):
        criteria = {name: 'label reads: ' + ', '.join(spellings)
                    for name, spellings in options.items()}
        criteria[NONE] = 'none of these bottles, or the text is too garbled to tell'
        answers, why = ask(
            {'ocr_text_of_one_liquor_bottle_label': text},
            {'brand': {'type': 'choice', 'criteria': criteria,
                       'instructions': 'Which bottle is this label from?'}})
        if answers is None:
            # A failed call only means Gemini gets the crop, as without Jev.
            print(why, file=sys.stderr)
            return None, 0.0
        answer = answers.get('brand') or {}
        if answer.get('choice') not in options:
            return None, 0.0
        return answer['choice'], round(float(answer['confidence']), 3)

    return jev
