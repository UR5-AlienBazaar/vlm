"""The drinks menu: one drink, one API call, a list of pendant scripts.

A drink is nothing more than the pipelines (scripts taught on the pendant)
to run for it, in order:

    drinks:
      whiskey_cola:
        name: Whiskey & Cola
        scripts: [grab_whiskey, pour_whiskey, return_whiskey,
                  grab_cola, pour_cola, return_cola]

The menu file only NAMES scripts. Teaching them is separate, and a drink
whose scripts are not all taught yet is listed as not ready and refused
before anything moves. That is how the menu can be written first and the
scripts filled in one by one.

Read fresh on every request, like the point file: editing the menu takes
effect on the next call, with no restart.
"""
import os

import yaml


class MenuError(Exception):
    """The menu file is missing or malformed."""


class Drink:
    """One entry on the menu."""

    def __init__(self, key, name, scripts, note=''):
        self.key = key
        self.name = name
        self.scripts = scripts
        self.note = note

    def missing(self, store):
        """Scripts this drink runs that are not taught, and points they lack."""
        gone = [s for s in self.scripts if s not in store.pipelines]
        points = []
        for s in self.scripts:
            if s in store.pipelines:
                points += [p for p in store.pipelines[s].missing_points(store)
                           if p not in points]
        return gone, points

    def to_json(self, store):
        scripts, points = self.missing(store)
        out = {'drink': self.key, 'name': self.name, 'scripts': self.scripts,
               'ready': not scripts and not points}
        if scripts:
            out['missing_scripts'] = scripts
        if points:
            out['missing_points'] = points
        if self.note:
            out['note'] = self.note
        return out


def load(path):
    """Return {key: Drink} from the menu YAML at `path`."""
    try:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
    except OSError as exc:
        raise MenuError(f'cannot read menu {path}: {exc.strerror}') from exc
    except yaml.YAMLError as exc:
        raise MenuError(f'{path} is not valid YAML: {exc}') from exc
    drinks = raw.get('drinks') if isinstance(raw, dict) else None
    if not isinstance(drinks, dict):
        raise MenuError(f'{path}: expected a `drinks:` mapping')
    out = {}
    for key, d in drinks.items():
        key = str(key)
        if not isinstance(d, dict):
            raise MenuError(f'{path}: drink {key!r} is not a mapping')
        scripts = d.get('scripts')
        if (not isinstance(scripts, list) or not scripts
                or not all(isinstance(s, str) and s for s in scripts)):
            raise MenuError(
                f'{path}: drink {key!r} needs `scripts:`, a non-empty list '
                f'of pipeline names')
        out[key] = Drink(key, str(d.get('name') or key), scripts,
                         str(d.get('note') or ''))
    return out


def menu_path_for(arg):
    """Resolve a `--menu` argument, the same way `--points` is resolved.

    A bare name such as `workcell` means bartender_teach/config/
    workcell_menu.yaml, beside workcell_points.yaml and preferring the
    source tree to the install. Anything with a slash or a .yaml suffix is
    a path. None means no menu.
    """
    from bartender_teach.point_store import default_points_path
    if arg is None:
        return None
    if os.sep in arg or arg.endswith(('.yaml', '.yml')):
        return os.path.abspath(os.path.expanduser(arg))
    return default_points_path(f'{arg}_menu.yaml')
