"""Tests for menu.py: loading and checking the drinks menu file."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api import menu                              # noqa: E402


def _load(tmp_path, text):
    path = tmp_path / 'menu.yaml'
    path.write_text(text)
    return menu.load(str(path))


def test_name_defaults_to_the_key(tmp_path):
    drinks = _load(tmp_path, 'drinks:\n  gin_fanta:\n    scripts: [a]\n')
    assert drinks['gin_fanta'].name == 'gin_fanta'


@pytest.mark.parametrize('text', [
    'nothing: here\n',
    'drinks: [a, b]\n',
    'drinks:\n  x: 3\n',
    'drinks:\n  x:\n    scripts: []\n',
    'drinks:\n  x:\n    scripts: [a, 5]\n',
    'drinks: [unclosed\n',
])
def test_a_malformed_menu_is_refused(tmp_path, text):
    with pytest.raises(menu.MenuError):
        _load(tmp_path, text)


def test_a_missing_menu_is_refused(tmp_path):
    with pytest.raises(menu.MenuError):
        menu.load(str(tmp_path / 'nope.yaml'))


def test_menu_path_for_keeps_paths_and_has_no_default():
    assert menu.menu_path_for(None) is None
    assert menu.menu_path_for('/x/y.yaml') == '/x/y.yaml'
    assert menu.menu_path_for('workcell').endswith('workcell_menu.yaml')
