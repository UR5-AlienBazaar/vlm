"""The error taxonomy: turning a skill's free-text failure into a code.

See docs/CONTROL_API.md for the full design. Every code here is drawn from
a real failure this project has produced, and the classifier below matches
against the RESULT MESSAGE a skill already returns -- because, today, that
is genuinely all that reaches this layer.

THE GAP THIS EXPOSED. open_action_server's richest failures (a grasp that
stalled early, a plan that only reached part of its path) are logged in
detail by Arm._error and then, at the action boundary, folded into a
coarse wrapper: "arm B could not pick up the opener". Two of this
project's most carefully separated faults -- GRIPPER_NOT_FOLLOWING and
GRASP_STOPPED_WIDE, split apart specifically because conflating them sent
one investigation to the wrong half of the robot -- used to collapse back
into that single sentence the moment they reached here. Arm.last_error and
OpenActionServer._why/_why_any (see open_action_server.py) now thread the
specific reason back into the message this classifier reads, so STAGE_FAILED
below is the exception rather than the common case for open_bottle.

pour_action_server now does the same thing (PourActionServer.last_error /
_why, added after the note above was first written), but its sub-calls use
different wording than open_bottle's, so most of them still fall through to
STAGE_FAILED rather than a specific code -- the classifier now sees the
reason (it lands in `detail`), it just does not always recognise it as one
of the named faults yet. Two of pour's phrasings happen to already match:
`_move_cartesian`'s "only reached X of the path" is PLAN_FAILED, and
`_still_holding_bottle`'s "lost: fingers have closed to" is GRASP_LOST.
Widening the rest is future work, added rule by rule as pour actually
produces the message rather than guessed in advance -- a wrong code is
worse than an honest STAGE_FAILED, which is what classify() falls back to.
"""
import re

# code -> (meaning, default retryable)
#
# The thirteen from CONTROL_API.md, plus two additions this classifier
# needs and the doc does not name: a message that names a stage but no
# specific reason (STAGE_FAILED), and one this classifier does not
# recognise at all (UNCLASSIFIED). Both default to a conservative answer
# for `retryable` rather than a guess -- see the docstring on classify().
TAXONOMY = {
    'UNKNOWN_STATION': ('no such bottle/glass/slot', False),
    'OUT_OF_APPROACH_WINDOW': ('geometry says no arm can side-grasp it', False),
    'OUT_OF_REACH': ("past the arm's radius", False),
    'NO_ARM_CAN_DO_BOTH': (
        'one arm reaches A, another reaches B, neither both', False),
    'ALREADY_OPEN': (
        "premise wrong; the cap is off and can't re-attach in sim", False),
    'PLAN_FAILED': ('MoveIt found nothing', True),
    'MOVE_STOPPED_SHORT': ('controller said success, flange is elsewhere', True),
    'GRIPPER_NOT_FOLLOWING': ('goal accepted, joint never moved', False),
    'GRASP_STOPPED_WIDE': ('fingers closed fine, then met the wrong thing', True),
    'GRASP_LOST': ('had it, dropped it mid-sequence', True),
    'OBJECT_NOT_SEATED': ('got there, geometry check failed', True),
    'OBJECT_DISTURBED': ('the workpiece moved more than allowed', True),
    'SCENE_STALE': ('no model poses; the bridge is down', False),
    'STAGE_FAILED': (
        'a named stage did not complete; no finer reason reached this '
        'layer, so check the server log', True),
    'UNCLASSIFIED': (
        'a failure message this classifier does not recognise', False),
}

_MM = r'(-?[\d.]+)\s*mm'


def _measurements(**kv):
    return {k: v for k, v in kv.items() if v is not None}


# Ordered rules: (compiled pattern, code, measurement-group extractor).
# First match wins, and order matters -- OBJECT_NOT_SEATED's pattern would
# also match a bare "off centre" fragment inside a longer sentence, so the
# more specific patterns run first.
def _rule(pattern, code, extract=None):
    return (re.compile(pattern), code, extract or (lambda m: {}))


_RULES = (
    _rule(
        r'^no capped bottle called|^no such bottle/glass/slot',
        'UNKNOWN_STATION',
    ),
    _rule(r'no model poses on .* is the ros_gz bridge', 'SCENE_STALE'),
    _rule(r'lost sight of the cap', 'SCENE_STALE'),
    _rule(r"already open .*cannot re-attach", 'ALREADY_OPEN'),
    _rule(
        r'is not seated on the cap: ' + _MM + r' down \(needs ([\d.]+)\) and '
        r'' + _MM + r' off centre \(allows ([\d.]+)\)',
        'OBJECT_NOT_SEATED',
        lambda m: _measurements(
            depth_mm=float(m.group(1)), depth_required_mm=float(m.group(2)),
            offset_mm=float(m.group(3)), offset_allowed_mm=float(m.group(4)),
        ),
    ),
    _rule(
        r'moved ' + _MM + r' while being pushed on \(limit ' + _MM + r'\)',
        'OBJECT_DISTURBED',
        lambda m: _measurements(
            shift_mm=float(m.group(1)), shift_limit_mm=float(m.group(2)),
        ),
    ),
    _rule(
        r'is not holding the beer any more|had it, dropped it|'
        r'lost: fingers have closed to',
        'GRASP_LOST',
    ),
    _rule(r'it is the gripper not following', 'GRIPPER_NOT_FOLLOWING'),
    _rule(
        r'so they are on something else and the arm is probably not where',
        'GRASP_STOPPED_WIDE',
    ),
    _rule(
        r'only reached ([\d.]+) of the path',
        'PLAN_FAILED',
        lambda m: _measurements(fraction_reached=float(m.group(1))),
    ),
    _rule(
        r'flange is ' + _MM + r' from where it was sent',
        'MOVE_STOPPED_SHORT',
        lambda m: _measurements(offset_mm=float(m.group(1))),
    ),
)


def classify(message, stage=None):
    """Turn a skill's free-text `message` into a taxonomy entry.

    Returns a dict with `code`, `detail` (the original message), `stage`
    (if given), `measurements` (only the ones the message actually carried
    -- never invented), `retryable`, and a `suggest` string. Matching is
    ordered and stops at the first hit; an unrecognised message becomes
    UNCLASSIFIED rather than a guess, because a wrong code is worse than an
    honest "do not know" -- exactly the reasoning that split
    GRIPPER_NOT_FOLLOWING from GRASP_STOPPED_WIDE in the first place.
    """
    for pattern, code, extract in _RULES:
        found = pattern.search(message)
        if found:
            measurements = extract(found)
            return _entry(code, message, stage, measurements)
    if _looks_like_a_stage_summary(message):
        return _entry('STAGE_FAILED', message, stage, {})
    return _entry('UNCLASSIFIED', message, stage, {})


def _looks_like_a_stage_summary(message):
    """Report whether `message` is a coarse "could not X" wrapper.

    These are real, current messages (see open_action_server.py's
    execute_callback and pour_action_server.py's execute_callback) that
    name a stage without a specific reason reaching this layer -- as
    opposed to a message this classifier has simply never seen before,
    which is UNCLASSIFIED and a stronger signal that something new broke.
    """
    return bool(re.search(
        r'could not |would not |did not run|failed at step|failed while '
        r'pouring|could not stow', message))


def _entry(code, message, stage, measurements):
    meaning, retryable = TAXONOMY[code]
    out = {
        'code': code,
        'detail': message,
        'retryable': retryable,
        'suggest': _suggest(code, meaning),
    }
    if stage is not None:
        out['stage'] = stage
    if measurements:
        out['measurements'] = measurements
    return out


def _suggest(code, meaning):
    if code == 'GRIPPER_NOT_FOLLOWING':
        return 'restart the simulator; this does not clear on its own'
    if code in ('UNCLASSIFIED',):
        return 'no taxonomy match -- read the message and, if it recurs, add a rule'
    if code == 'STAGE_FAILED':
        return ('retry once; if it fails three times in a row restart the '
                'stack rather than continue retrying')
    return f'{meaning} -- retry' if TAXONOMY[code][1] else meaning
