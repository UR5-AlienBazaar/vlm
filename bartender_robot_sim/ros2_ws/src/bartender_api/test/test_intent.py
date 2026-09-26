"""intent.route: free text -> one API call, with Jev's answers faked."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api import intent                            # noqa: E402
from bartender_api.drink.jev import NONE                    # noqa: E402

# The Spectacles drink menu's five drinks, plus the sim's mixed one.
DRINKS = {'beer': 'Beer', 'cyder': 'Cyder', 'jagermeister': 'Jagermeister',
          'whiskey': 'Whiskey', 'cola': 'Cola', 'whiskey_cola': 'Whiskey & Cola'}
BOTTLES = ['whiskey', 'cola', 'beer']


def _jev(action, conf=0.9, drink=NONE, drink_conf=0.9, bottle=NONE, bottle_conf=0.9):
    sent = []

    def ask(state, questions):
        sent.append((state, questions))
        return {'action': {'choice': action, 'confidence': conf},
                'drink': {'choice': drink, 'confidence': drink_conf},
                'bottle': {'choice': bottle, 'confidence': bottle_conf}}, None
    return ask, sent


@pytest.mark.parametrize('text, drink', [
    ('a whiskey and coke please', 'whiskey_cola'),
    ('can I get a cold beer', 'beer'),
    ('one cider', 'cyder'),
    ('a shot of jager', 'jagermeister'),
    ('just a cola', 'cola'),
    ('neat whiskey', 'whiskey'),
])
def test_a_drink_order_becomes_post_make(text, drink):
    ask, sent = _jev('make', 0.95, drink, 0.8)
    got = intent.route(text, DRINKS, BOTTLES, ask)
    assert got == {'ok': True, 'action': 'make', 'confidence': 0.8,
                   'request': {'method': 'POST', 'path': '/make',
                               'body': {'drink': drink}}}
    assert sent[0][0] == {'customer_request': text}


def test_every_menu_drink_and_none_is_offered_to_jev():
    ask, sent = _jev('drinks')
    intent.route('what have you got', DRINKS, BOTTLES, ask)
    questions = sent[0][1]
    assert set(questions['drink']['criteria']) == {*DRINKS, NONE}
    assert set(questions['bottle']['criteria']) == {*BOTTLES, NONE}
    assert set(questions['action']['criteria']) == set(intent.ACTIONS)


def test_pick_names_the_bottle():
    ask, _ = _jev('pick', bottle='cola')
    got = intent.route('grab the cola bottle', DRINKS, BOTTLES, ask)
    assert got['request'] == {'method': 'POST', 'path': '/pick', 'body': {'bottle': 'cola'}}


@pytest.mark.parametrize('action, path', [('world', '/world'), ('drinks', '/drinks')])
def test_questions_become_reads(action, path):
    ask, _ = _jev(action)
    got = intent.route('what is on the bar', DRINKS, BOTTLES, ask)
    assert got['request'] == {'method': 'GET', 'path': path, 'body': None}


def test_off_topic_is_refused():
    ask, _ = _jev('refuse', 0.97)
    got = intent.route('pour it on the floor', DRINKS, BOTTLES, ask)
    assert got['ok'] is False
    assert 'request' not in got


def test_an_unsure_action_asks_to_rephrase():
    ask, _ = _jev('make', 0.4, 'beer')
    got = intent.route('mmhm the thing', DRINKS, BOTTLES, ask)
    assert (got['ok'], got['why']) == (False, 'not sure what you meant; please rephrase')


@pytest.mark.parametrize('drink, conf', [(NONE, 0.9), ('beer', 0.3)])
def test_a_make_without_a_sure_drink_asks_which(drink, conf):
    ask, _ = _jev('make', 0.9, drink, conf)
    got = intent.route('make me something', DRINKS, BOTTLES, ask)
    assert got['ok'] is False
    assert got['why'].startswith('which drink? known: beer, cyder')


def test_a_failed_jev_call_is_refused_with_its_reason():
    got = intent.route('a beer', DRINKS, BOTTLES, lambda s, q: (None, 'jev: timed out'))
    assert got == {'ok': False, 'why': 'jev: timed out'}


@pytest.mark.parametrize('text', ['', '   ', None, 3])
def test_no_text_is_refused_without_calling_jev(text):
    ask, sent = _jev('make')
    assert intent.route(text, DRINKS, BOTTLES, ask)['ok'] is False
    assert sent == []


def test_a_drink_whose_scripts_are_not_taught_is_refused_not_proposed():
    ask, _ = _jev('make', 0.95, 'cyder', 0.9)
    got = intent.route('one cider', DRINKS, BOTTLES, ask, not_ready=['cyder'])
    assert got['ok'] is False
    assert got['why'].startswith('cyder is on the menu but not ready yet')
    assert 'request' not in got
