"""POST /ask: map a free-text request onto one existing API call, with Jev.

One Jev call asks three typed questions at once: which action, which drink
off the menu, which pickable bottle. The answer is the request to send
(method, path, body) and never runs it: the caller, or a person, does. An
unsure answer is refused with a reason rather than guessed, so the worst a
misread can do is ask the user to rephrase.

/can is not offered: it needs a station and a glass, which free text rarely
names reliably; add it when a caller needs it.
"""
from .drink.jev import NONE

# ponytail: calibration knob; below this Jev's pick is "please rephrase", not a call.
MIN_CONFIDENCE = 0.6

ACTIONS = {
    'world': 'asks what is on the bar, what the robot can see or reach',
    'drinks': 'asks what drinks are on the menu or can be made',
    'make': 'wants a drink made, poured or served',
    'pick': 'wants the robot to pick up or grab one bottle, not make a drink',
    'refuse': 'anything else: not about the bar, unsafe, or not something '
              'this robot does',
}
REQUESTS = {
    'world': ('GET', '/world'),
    'drinks': ('GET', '/drinks'),
    'make': ('POST', '/make'),
    'pick': ('POST', '/pick'),
}


def _choices(named, what):
    out = {key: what + ' ' + name for key, name in named.items()}
    out[NONE] = f'no {what} named, or not one of these'
    return out


def questions(drinks, bottles):
    """The three Jev questions, over the menu {key: name} and the pickable bottles."""
    return {
        'action': {'type': 'choice', 'criteria': ACTIONS,
                   'instructions': 'What does the customer want the bartending robot to do?'},
        'drink': {'type': 'choice', 'criteria': _choices(drinks, 'the drink'),
                  'instructions': 'Which drink on the menu do they mean?'},
        'bottle': {'type': 'choice',
                   'criteria': _choices({b: b for b in bottles}, 'the bottle'),
                   'instructions': 'Which bottle do they mean?'},
    }


def _refuse(why, answers=None):
    out = {'ok': False, 'why': why}
    if answers is not None:
        out['answers'] = answers
    return out


def route(text, drinks, bottles, ask, not_ready=()):
    """Return {"ok", "action", "confidence", "request": {method, path, body}} or a refusal.

    A drink in `not_ready` (its scripts are not all taught) is refused here
    rather than proposed, since /make would only refuse it.
    """
    if not isinstance(text, str) or not text.strip():
        return _refuse('ask needs some text')
    answers, why = ask({'customer_request': text.strip()}, questions(drinks, bottles))
    if answers is None:
        return _refuse(why)
    action = answers['action']
    confidence = action['confidence']
    if confidence < MIN_CONFIDENCE:
        return _refuse('not sure what you meant; please rephrase', answers)
    if action['choice'] == 'refuse':
        return _refuse('not something this bar does', answers)
    method, path = REQUESTS[action['choice']]
    body = None
    if action['choice'] in ('make', 'pick'):
        field = 'drink' if action['choice'] == 'make' else 'bottle'
        picked = answers[field]
        known = list(drinks) if field == 'drink' else list(bottles)
        if picked['choice'] == NONE or picked['confidence'] < MIN_CONFIDENCE:
            return _refuse(f'which {field}? known: {", ".join(known) or "none"}', answers)
        if picked['choice'] in not_ready:
            return _refuse(f'{picked["choice"]} is on the menu but not ready yet: '
                           'its scripts are not all taught', answers)
        body = {field: picked['choice']}
        confidence = min(confidence, picked['confidence'])
    return {'ok': True, 'action': action['choice'], 'confidence': round(confidence, 3),
            'request': {'method': method, 'path': path, 'body': body}}
