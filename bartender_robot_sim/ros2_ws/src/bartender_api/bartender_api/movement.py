"""Phase C, scoped to simple movement: goto / jog / gripper over Pendant.

Wraps `Pendant.dispatch()` rather than reimplementing it -- see
docs/CONTROL_API.md's "The movement API should wrap Pendant.dispatch(), not
reimplement it." That is what already encodes every bound this project has
found a reason for: MAX_JOG_MM/MAX_JOG_DEG (refuse, don't clamp), the
gripper's refusal to park on its lower limit, and `goto` always driving the
arm a point was taught on rather than whichever one is selected.

`dispatch()` itself has no structured result -- it prints, and the only
line shapes it is known to produce are documented on `_classify` below.
This module pre-validates everything it can (axis name, jog bound, gripper
bound, point existence) so dispatch is only ever called with something it
should accept, which is what keeps `_classify`'s text-matching honest
rather than a guess.

Two absolute-target primitives from docs/CONTROL_API.md's sketch --
`/move/joints` (an arbitrary joint vector) and `/move/tool` (an arbitrary
Cartesian pose) -- are deliberately NOT built here. Pendant has no verb for
either; `goto` only drives to a taught point and `jog` only moves
relative to where the arm already is. Adding them would mean either a new
Pendant command (real work, not done here) or calling TeachNode's
move_to_joints/move_cartesian directly and losing every bound dispatch
provides for free. "Simple movement for now" is goto, jog, and the
gripper -- the three things Pendant already does safely.

PICK is the one exception to "taught points only, one move at a time", and
it is still taught: `pick(bottle)` replays the grab_<bottle> pipeline that
someone recorded on the pendant (`run grab_<bottle>`), nothing more. Which
bottles exist is whatever grab_* pipelines the point file has, so adding a
bottle is teaching it, not changing this code.

MAKE is the same thing one level up: a drink on the menu (menu.py) is a
list of those taught scripts, run in order as one command, stopping at the
first that does not finish.
"""
import contextlib
import io
import threading

from bartender_teach.point_store import PointStore, PointStoreError
from bartender_teach.teach_points import (
    ARMS, GRIPPER_OPEN_POS, GRIPPER_UPPER_LIMIT, MAX_JOG_DEG, MAX_JOG_MM,
    Pendant,
)

from . import menu

# j1..j6 are handled separately (any of the arm's six joints); these are the
# rest of what Pendant.cmd_jog recognises -- see AXES and cmd_jog itself.
_LINEAR_AXES = {'x', 'y', 'z', 'tx', 'ty', 'tz'}
_ROTATION_AXES = {'rx', 'ry', 'rz'}

# A bottle is pickable when the point file has a pipeline called
# grab_<bottle>, taught on the pendant with `record grab_<bottle>`. Nothing
# else lists the bottles: teaching grab_gin is what makes `gin` a choice.
GRAB_PREFIX = 'grab_'


def _bottle_names(pipelines):
    return [name[len(GRAB_PREFIX):] for name in sorted(pipelines)
            if name.startswith(GRAB_PREFIX) and len(name) > len(GRAB_PREFIX)]


def _is_joint_axis(axis):
    return axis.startswith('j') and axis[1:].isdigit() and 1 <= int(axis[1:]) <= 6


def _jog_limit(axis):
    """(limit, unit) for `axis` -- MAX_JOG_MM for linear, MAX_JOG_DEG else.

    Mirrors cmd_jog's own branches exactly, against the same two constants,
    so a request this rejects is one dispatch would also have refused.
    """
    if axis in _LINEAR_AXES:
        return MAX_JOG_MM, 'mm'
    return MAX_JOG_DEG, 'deg'


def _refuse(message):
    return {'ok': False, 'message': message}


def _classify(text):
    """Report whether dispatch's captured output means the command worked.

    dispatch() has no structured (ok, message) result at its own boundary
    -- see its docstring -- so this reads the two shapes its call sites
    actually produce: Pendant._report's "FAILED: {why}" for a motion that
    was attempted and failed, and _require_pose's "cannot read ... through
    /compute_fk" for the one runtime refusal this module does not
    pre-validate away (move_group not running). Everything else dispatch
    could refuse -- an unknown axis, a jog past its bound, an unknown point,
    a gripper position off its band -- is caught before dispatch ever runs;
    see goto()/jog()/gripper() below. An empty buffer (should not happen if
    a line was actually dispatched) is treated as failure rather than a
    silent success.
    """
    if not text.strip():
        return False
    return 'FAILED:' not in text and 'cannot read' not in text


class MovementBridge:
    """Runs Pendant commands for the HTTP layer, one at a time.

    Mirrors teach_gui.py's Bridge exactly, for the same reason:
    dispatch() blocks the calling thread until the robot finishes moving,
    and two motion goals interleaved on one arm is the failure this exists
    to prevent. This bridge and the browser pendant's each hold their own
    lock in their own process, so the one thing this does NOT prevent is
    the browser pendant and this API racing each other on the same arm --
    a real, known gap, no different from the one that already exists
    between the browser and terminal pendants.
    """

    def __init__(self, node, store, menu_path=None):
        self.node = node
        self.pendant = Pendant(node, store)
        self.menu_path = menu_path
        self._lock = threading.Lock()

    def _run(self, arm, line):
        if arm not in ARMS:
            return _refuse(f"no arm {arm!r}; known: {', '.join(ARMS)}")
        if not self._lock.acquire(blocking=False):
            return _refuse('busy: a command is already running')
        try:
            text = self._dispatch(f'arm {arm}', line)
        finally:
            self._lock.release()
        if isinstance(text, dict):
            return text
        return {'ok': _classify(text), 'message': text.strip()}

    def _dispatch(self, *lines):
        """Run pendant lines with the lock held; their output, or a refusal."""
        try:
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                for line in lines:
                    self.pendant.dispatch(line)
            return buf.getvalue()
        except Exception as exc:                            # noqa: BLE001
            # dispatch() already swallows the failures it knows about;
            # anything reaching here is a bug and must not take the whole
            # server down with it.
            self.node.get_logger().error(
                f'movement command {lines[-1]!r} raised: {exc}')
            return {'ok': False,
                    'message': f'unexpected error: {type(exc).__name__}: {exc}'}

    def _reload(self):
        """Re-read the point file, so bottles taught since startup show up.

        The pendant that teaches them is another process writing the same
        file; without this a new grab_gin would need a server restart.
        Returns None, or why the file could not be read (the old contents
        are kept then).
        """
        try:
            self.pendant.store = PointStore.load(self.pendant.store.path)
        except PointStoreError as exc:
            return str(exc)
        return None

    def bottles(self):
        """Return the pickable bottles: one per grab_<bottle> pipeline."""
        with self._lock:
            why = self._reload()
            pipelines = self.pendant.store.pipelines
        found = [{'bottle': bottle, 'pipeline': GRAB_PREFIX + bottle,
                  'steps': len(pipelines[GRAB_PREFIX + bottle])}
                 for bottle in _bottle_names(pipelines)]
        out = {'bottles': found, 'points_file': self.pendant.store.path}
        if why:
            out['warning'] = f'point file not re-read: {why}'
        return out

    def pick(self, bottle):
        """Run the grab_<bottle> pipeline, start to finish."""
        if not isinstance(bottle, str) or not bottle.strip():
            return _refuse('pick needs a bottle name')
        bottle = bottle.strip().lower()
        if not self._lock.acquire(blocking=False):
            return _refuse('busy: a command is already running')
        try:
            why = self._reload()
            if why:
                return _refuse(f'cannot read the point file: {why}')
            name = GRAB_PREFIX + bottle
            try:
                pipeline = self.pendant.store.pipelines[name]
            except KeyError:
                known = sorted(n[len(GRAB_PREFIX):]
                               for n in self.pendant.store.pipelines
                               if n.startswith(GRAB_PREFIX))
                return _refuse(
                    f'no bottle {bottle!r} to pick: there is no {name} '
                    f'pipeline. Known: {", ".join(known) or "none yet"}. '
                    f'Teach one on the pendant with `record {name}`.')
            missing = pipeline.missing_points(self.pendant.store)
            if missing:
                return _refuse(f'{name} names points that do not exist: '
                               f'{", ".join(missing)}')
            result = self._run_scripts([name])
        finally:
            self._lock.release()
        result['bottle'] = bottle
        return result

    def _run_scripts(self, names):
        """Run pipelines one after another, lock held; stop at the first failure.

        Not _classify: `run` says "finished NAME" only when every step of
        NAME worked, and anything else (a STOPPED line, a freedrive
        refusal) means it did not happen, so the next one must not start.
        """
        log = []
        for index, name in enumerate(names, 1):
            text = self._dispatch(f'run {name}')
            if isinstance(text, dict):
                text['message'] = '\n'.join(log + [text['message']])
                return text
            log.append(text.strip())
            if f'finished {name}' not in text:
                out = {'ok': False, 'message': '\n'.join(log)}
                if len(names) > 1:
                    out['stopped_at'] = f'script {index} of {len(names)}: {name}'
                return out
        return {'ok': True, 'message': '\n'.join(log)}

    def _menu(self):
        """Return ({key: Drink}, None) or (None, why there is no menu)."""
        if self.menu_path is None:
            return None, ('no menu: start the server with --menu (e.g. '
                          '--points workcell, which also picks workcell_menu.yaml)')
        try:
            return menu.load(self.menu_path), None
        except menu.MenuError as exc:
            return None, str(exc)

    def choices(self):
        """Return ({drink key: name}, [drink not ready], [bottle]) for /ask, without the lock.

        Read straight from disk into a store of its own: drinks() and
        bottles() take the command lock, which /make holds for a whole drink,
        so asking during one would hang until the drink was done.
        """
        drinks, _ = self._menu()
        drinks = drinks or {}
        path = self.pendant.store.path
        try:
            store = PointStore.load(path)
        except PointStoreError:
            store = PointStore(path)   # empty: no bottle to pick, no drink ready
        return ({key: d.name for key, d in drinks.items()},
                [key for key, d in drinks.items() if any(d.missing(store))],
                _bottle_names(store.pipelines))

    def drinks(self):
        """Return the menu, each drink saying whether its scripts are taught."""
        with self._lock:
            why_points = self._reload()
            drinks, why = self._menu()
            store = self.pendant.store
        if drinks is None:
            return {'drinks': [], 'error': why}
        out = {'drinks': [d.to_json(store) for d in drinks.values()],
               'menu_file': self.menu_path}
        if why_points:
            out['warning'] = f'point file not re-read: {why_points}'
        return out

    def make(self, drink):
        """Run every script of one drink, in order, as one command."""
        if not isinstance(drink, str) or not drink.strip():
            return _refuse('make needs a drink name')
        drink = drink.strip().lower()
        if not self._lock.acquire(blocking=False):
            return _refuse('busy: a command is already running')
        try:
            why = self._reload()
            if why:
                return _refuse(f'cannot read the point file: {why}')
            drinks, why = self._menu()
            if drinks is None:
                return _refuse(why)
            entry = drinks.get(drink)
            if entry is None:
                return _refuse(f'no drink {drink!r} on the menu. Known: '
                               f'{", ".join(drinks) or "none"}.')
            # All of it checked before the first script moves anything:
            # finding out at script 4 leaves the arm holding a bottle.
            scripts, points = entry.missing(self.pendant.store)
            if scripts or points:
                parts = []
                if scripts:
                    parts.append('scripts not taught yet: ' + ', '.join(scripts))
                if points:
                    parts.append('points missing: ' + ', '.join(points))
                return _refuse(f'{entry.name} is not ready -- '
                               + '; '.join(parts))
            result = self._run_scripts(entry.scripts)
        finally:
            self._lock.release()
        result['drink'] = drink
        return result

    def goto(self, arm, point):
        if not point:
            return _refuse('goto needs a point name')
        try:
            self.pendant.store.get(point)
        except PointStoreError as exc:
            return _refuse(str(exc))
        return self._run(arm, f'goto {point}')

    def jog(self, arm, axis, amount):
        axis = axis.lower() if isinstance(axis, str) else ''
        if not (axis in _LINEAR_AXES or axis in _ROTATION_AXES
                or _is_joint_axis(axis)):
            return _refuse(
                f'unknown jog axis {axis!r}. Expected j1..j6, x/y/z, '
                f'tx/ty/tz or rx/ry/rz.')
        try:
            amount = float(amount)
        except (TypeError, ValueError):
            return _refuse(f'jog amount must be a number, got {amount!r}')
        limit, unit = _jog_limit(axis)
        if abs(amount) > limit:
            return _refuse(
                f'{amount:g}{unit} exceeds the {limit:g}{unit} jog limit. '
                f'Refusing rather than clamping -- send it in steps if '
                f'that is really what you meant.')
        return self._run(arm, f'jog {axis} {amount:g}')

    def gripper(self, arm, position):
        try:
            position = float(position)
        except (TypeError, ValueError):
            return _refuse(
                f'gripper position must be a number, got {position!r}')
        if not GRIPPER_OPEN_POS <= position <= GRIPPER_UPPER_LIMIT:
            return _refuse(
                f'gripper position {position:.4f} is outside '
                f'{GRIPPER_OPEN_POS:.2f}..{GRIPPER_UPPER_LIMIT:.2f}; resting '
                f'on the lower joint limit stops it responding for the '
                f'rest of the run.')
        return self._run(arm, f'close {position:g}')
