"""Load, save and validate a file of taught robot points.

Kept separate from the teach pendant itself so that anything which CONSUMES
taught points -- pour_action_server, a test, a one-off script -- can read them
without pulling in the interactive tool or any of its action clients.

File format (YAML)::

    frame: base_link
    group: ur_manipulator
    eef_link: tool0
    points:
      whiskey_approach:
        joints: {shoulder_pan_joint: 0.079, shoulder_lift_joint: -1.905, ...}
        pose: {xyz: [...], quat_xyzw: [...]}   # FK at record time, informational
        gripper: 0.0
        note: above the whiskey, clear of its shoulder
        recorded: '2026-09-18T12:00:00'
      b_home:
        group: b_ur_manipulator                # <- arm B; see below
        joints: {b_shoulder_pan_joint: 0.0, ...}

The top-level `frame`/`group`/`eef_link` are the DEFAULT for points that do
not name their own, which is every point taught before there was a second
arm. A point may override `group`, and that is what makes one file able to
hold both arms.

Per-point rather than one file per arm: the two arms work the same bottle on
the same counter, and a point is only meaningful next to the others in the
scene. Splitting them would mean two files to keep in step and no way to ask
"what does the robot know about the beer station". Nothing here needs to know
what an arm IS -- it stores the group name the point was taught against and
hands it back; deciding what that group implies is the caller's business.

`joints` is a MAPPING, not a list, on purpose. /joint_states publishes the arm
and gripper joints in whatever order the controllers happen to register them,
and it is not the order MoveGroup wants them in. A list would record that
order silently and hand back a pose that looks plausible and moves the wrong
joints; a mapping cannot. `joints_in_order()` is the only way to get a list out
of here, and it takes the names it wants as an argument.
"""
import datetime
import math
import os
import tempfile

import yaml

from bartender_teach.pipelines import Pipeline, PipelineError

# Wrapping is not cosmetic. /compute_ik hands back arbitrary branches, often
# out near +/-2pi, which describe the same pose but force a long wrist sweep
# to reach -- pour_action_server has a note about exactly this dragging the
# gripper through forearm_link and being rejected as a self-collision. Points
# taught by driving the arm there by hand can pick up the same branches, so
# they are wrapped on the way in and the change is reported, never silent.
TWO_PI = 2.0 * math.pi


class PointStoreError(Exception):
    """A point file is malformed, or a point in it does not exist.

    Always carries the file path and what was wrong with it.
    """


def wrap_angle(a: float) -> float:
    """Wrap into [-pi, pi]."""
    return (float(a) + math.pi) % TWO_PI - math.pi


class Point:
    """One taught configuration."""

    def __init__(self, name, joints, pose=None, gripper=None, note='',
                 recorded=None, tool=None, group=None):
        self.name = name
        # Which planning group this was taught against, or None for "whatever
        # the file's default is". Kept as the group NAME and not an arm index
        # so that a third arm, or a renamed group, needs no change here.
        self.group = group or None
        self.joints = {str(k): float(v) for k, v in joints.items()}
        self.pose = pose
        self.gripper = None if gripper is None else float(gripper)
        # Which tool centre point was selected when this was taught. Recorded
        # for the reader's sake, not replayed: a point is a joint
        # configuration, and that is unambiguous whatever tool was selected.
        # But "spout_over_glass" taught with the whiskey spout selected means
        # something different from the same name taught at the flange, and
        # without this there is no way to tell them apart later.
        self.tool = tool
        self.note = note or ''
        self.recorded = recorded or datetime.datetime.now().isoformat(
            timespec='seconds')

    def joints_in_order(self, names):
        """Return joint values in the order `names` gives.

        Which order that is is the caller's business, not this file's.

        Raises rather than filling a gap with zero: a missing joint here means
        the point was taught against a different robot or a renamed joint, and
        a zero would be a perfectly plausible-looking value that drives the arm
        somewhere nobody asked for.
        """
        missing = [n for n in names if n not in self.joints]
        if missing:
            raise PointStoreError(
                f"point '{self.name}' has no value for {', '.join(missing)}; "
                f'it stores {", ".join(sorted(self.joints))}. It was probably '
                f'taught against a different robot or joint naming.')
        return [self.joints[n] for n in names]

    def wrapped(self):
        """Return a copy with every joint wrapped into [-pi, pi].

        Gives back (point, [(name, before, after), ...]) so the caller can
        report which joints actually moved.
        """
        changed, out = [], {}
        for name, value in self.joints.items():
            w = wrap_angle(value)
            out[name] = w
            if abs(w - value) > 1e-9:
                changed.append((name, value, w))
        return Point(self.name, out, self.pose, self.gripper, self.note,
                     self.recorded, self.tool, self.group), changed

    def to_dict(self):
        d = {'joints': {k: round(v, 6) for k, v in sorted(self.joints.items())},
             'recorded': self.recorded}
        if self.group:
            d['group'] = self.group
        if self.pose is not None:
            d['pose'] = self.pose
        if self.gripper is not None:
            d['gripper'] = round(self.gripper, 6)
        if self.tool:
            d['tool'] = self.tool
        if self.note:
            d['note'] = self.note
        return d

    @classmethod
    def from_dict(cls, name, d, path='<memory>'):
        if not isinstance(d, dict):
            raise PointStoreError(
                f"{path}: point '{name}' is {type(d).__name__}, expected a "
                f'mapping with at least a `joints:` key')
        joints = d.get('joints')
        if not isinstance(joints, dict) or not joints:
            raise PointStoreError(
                f"{path}: point '{name}' has no usable `joints:` mapping "
                f'(got {joints!r}). See the module docstring for the format.')
        for jname, jval in joints.items():
            if not isinstance(jval, (int, float)) or isinstance(jval, bool):
                raise PointStoreError(
                    f"{path}: point '{name}' joint '{jname}' is {jval!r}, "
                    f'expected a number in radians')
        group = d.get('group')
        if group is not None and not isinstance(group, str):
            raise PointStoreError(
                f"{path}: point '{name}' has group {group!r}, expected the "
                f'name of a planning group')
        return cls(name, joints, d.get('pose'), d.get('gripper'),
                   d.get('note', ''), d.get('recorded'), d.get('tool'), group)

    def describe(self, joint_order=None, default_group=None):
        """One line for a listing.

        `default_group` is the file's group. A point taught against anything
        else is tagged, because the same six numbers mean a different place in
        the room on a different arm, and a listing that does not say which arm
        is a listing you cannot act on.
        """
        names = joint_order or sorted(self.joints)
        if joint_order is None or not any(n in self.joints for n in names):
            names = sorted(self.joints)
        vals = ' '.join(f'{self.joints[n]:+.4f}' for n in names
                        if n in self.joints)
        line = f'{self.name:<24} [{vals}]'
        if self.group and self.group != default_group:
            line += f'  <{self.group}>'
        if self.pose:
            x, y, z = self.pose['xyz']
            line += f'  tool ({x:+.3f} {y:+.3f} {z:+.3f})'
        if self.gripper is not None:
            line += f'  grip {self.gripper:.3f}'
        if self.tool and self.tool != 'tool0':
            line += f'  [{self.tool}]'
        if self.note:
            line += f'\n{"":<24}  {self.note}'
        return line


class PointStore:
    """The whole file. Reads are strict; writes are atomic."""

    def __init__(self, path, frame='base_link', group='ur_manipulator',
                 eef_link='tool0'):
        self.path = path
        self.frame = frame
        self.group = group
        self.eef_link = eef_link
        self.points = {}
        # Pipelines live in the same file as the points they name. They are
        # a separate mapping rather than a kind of point because nothing that
        # only consumes points -- pour_action_server, open_action_server --
        # has any use for them, and an empty `pipelines:` key costs those
        # readers nothing.
        self.pipelines = {}

    # -- io ---------------------------------------------------------------

    @classmethod
    def load(cls, path, missing_ok=True):
        if not os.path.exists(path):
            if missing_ok:
                return cls(path)
            raise PointStoreError(f'{path}: no such point file')
        with open(path) as fh:
            try:
                raw = yaml.safe_load(fh)
            except yaml.YAMLError as exc:
                raise PointStoreError(f'{path}: not valid YAML: {exc}') from exc
        if raw is None:
            return cls(path)
        if not isinstance(raw, dict):
            raise PointStoreError(
                f'{path}: top level is {type(raw).__name__}, expected a mapping')

        store = cls(path,
                    raw.get('frame', 'base_link'),
                    raw.get('group', 'ur_manipulator'),
                    raw.get('eef_link', 'tool0'))
        points = raw.get('points') or {}
        if not isinstance(points, dict):
            raise PointStoreError(
                f'{path}: `points:` is {type(points).__name__}, expected a '
                f'mapping of name -> point')
        for name, d in points.items():
            store.points[str(name)] = Point.from_dict(str(name), d, path)

        pipelines = raw.get('pipelines') or {}
        if not isinstance(pipelines, dict):
            raise PointStoreError(
                f'{path}: `pipelines:` is {type(pipelines).__name__}, '
                f'expected a mapping of name -> pipeline')
        for name, d in pipelines.items():
            # Surfaced as a PointStoreError so that every caller which
            # already handles a malformed point file handles a malformed
            # pipeline too, rather than dying on an exception type it has
            # never heard of. The message is the pipeline module's.
            try:
                store.pipelines[str(name)] = Pipeline.from_dict(
                    str(name), d, path)
            except PipelineError as exc:
                raise PointStoreError(str(exc)) from None
        return store

    def save(self):
        """Write the file, atomically.

        Via a temp file and os.replace because this is the one artefact of a
        teaching session that cannot be reproduced by re-running anything. A
        crash or a full disk partway through a plain write would leave a
        truncated file, and the next load would reject it -- losing points that
        took somebody standing at the robot to record.
        """
        body = {
            'frame': self.frame,
            'group': self.group,
            'eef_link': self.eef_link,
            'points': {n: p.to_dict() for n, p in sorted(self.points.items())},
        }
        # Omitted entirely when there are none, so a workspace that never
        # records a pipeline keeps the file it had and the diff stays empty.
        if self.pipelines:
            body['pipelines'] = {
                n: p.to_dict() for n, p in sorted(self.pipelines.items())}
        directory = os.path.dirname(os.path.abspath(self.path)) or '.'
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=directory, suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as fh:
                fh.write(HEADER)
                yaml.safe_dump(body, fh, default_flow_style=False,
                               sort_keys=False, width=100)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self.path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
        return self.path

    # -- contents ---------------------------------------------------------

    def __contains__(self, name):
        return name in self.points

    def __len__(self):
        return len(self.points)

    def names(self, group=None):
        """Point names, or only those belonging to `group`.

        A point with no group of its own belongs to the file's default, which
        is how every point taught before there was a second arm keeps working.
        """
        if group is None:
            return sorted(self.points)
        return sorted(n for n, p in self.points.items()
                      if self.group_of(p) == group)

    def group_of(self, point):
        """Return the group a point belongs to, defaulting to the file's."""
        return point.group or self.group

    def get(self, name):
        try:
            return self.points[name]
        except KeyError:
            known = ', '.join(self.names()) or '(none)'
            raise PointStoreError(
                f"no point named '{name}' in {self.path}. Known: {known}"
            ) from None

    def add(self, point, overwrite=False):
        if point.name in self.points and not overwrite:
            raise PointStoreError(
                f"'{point.name}' already exists in {self.path}; pass "
                f'overwrite, or use a different name')
        self.points[point.name] = point
        return point

    def remove(self, name):
        self.get(name)
        return self.points.pop(name)


HEADER = """\
# Robot points taught with `ros2 run bartender_teach teach`.
#
# Angles are RADIANS. `joints` is a mapping so that joint order can never be
# lost; `pose` is the forward kinematics at the moment the point was recorded
# and is informational only -- nothing replays it, so editing it does nothing.
#
# Safe to hand-edit. The tool rewrites this file wholesale on save, so
# comments added below this header will not survive a save from the tool.
"""


def _source_config_beside(share_dir, filename='taught_points.yaml'):
    """Map an installed share directory back to its source tree.

    Returns None if that tree is no longer on disk.

    `<ws>/install/bartender_teach/share/bartender_teach` should resolve to
    `<ws>/src/bartender_teach/config`. Returns None when it cannot, which is
    the normal answer for an installed-only deployment.
    """
    d = os.path.abspath(share_dir)
    for _ in range(6):
        d = os.path.dirname(d)
        if not d or d == os.path.dirname(d):
            return None
        candidate = os.path.join(d, 'src', 'bartender_teach', 'config')
        if os.path.isdir(candidate):
            return os.path.join(candidate, filename)
    return None


def default_points_path(filename='taught_points.yaml'):
    """Where taught points live, preferring the SOURCE tree to the install.

    This ordering exists because of the obvious way to lose a teaching
    session: teach a dozen points, have them written into
    `install/bartender_teach/share/...`, run `colcon build`, and watch the
    install directory be repopulated from source. The points are gone, and the
    only copy was the one a person made by driving the robot. So if the source
    tree that this install was built from is still on disk, that is what gets
    written, and a rebuild then COPIES the points forward instead of over
    them.

    $BARTENDER_POINTS overrides everything, for a scratch file or a second
    workspace. `filename` picks another file in the same config directory
    (see points_path_for).
    """
    env = os.environ.get('BARTENDER_POINTS')
    if env and filename == 'taught_points.yaml':
        return os.path.abspath(os.path.expanduser(env))

    share = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('bartender_teach')
    except Exception:       # not built, or not sourced -- fall through
        share = None

    if share:
        return (_source_config_beside(share, filename)
                or os.path.join(share, 'config', filename))

    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'config', filename))


def points_path_for(arg):
    """Resolve a `--file` argument.

    A bare name such as `workcell` means config/workcell_points.yaml, next to
    taught_points.yaml and with the same source-before-install preference,
    so each cell keeps its own points without anyone typing the path into
    the source tree. Anything with a slash or a .yaml/.yml suffix is a path.
    None means the default file.
    """
    if arg is None:
        return default_points_path()
    if os.sep in arg or arg.endswith(('.yaml', '.yml')):
        return os.path.abspath(os.path.expanduser(arg))
    return default_points_path(f'{arg}_points.yaml')
