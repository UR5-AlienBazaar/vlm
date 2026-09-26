"""Ordered sequences of taught points, and the steps they are made of.

A point says where the arm can be. A pipeline says what order to visit points
in, and what to do with the gripper on the way -- which is the other half of
describing a skill, and the half that used to exist only as Python in an
action server.

Kept beside point_store rather than inside it because they are different
things with different lifetimes: a point is a measurement of the robot, a
pipeline is a plan made out of measurements. They share a file (see
PointStore) because a pipeline is meaningless without the points it names,
and two files that had to be kept in step would be a way to lose one.

File format, under the same top level as `points:`::

    pipelines:
      pour_whiskey:
        note: spirit first, mixer second
        recorded: '2026-09-19T20:00:00'
        steps:
          - {goto: whiskey_approach}
          - {grip: 0.25, arm: a, note: clamp past first contact}
          - {goto: whiskey_pour}
          - {wait: 0.5}
          - {goto: home}

WHY STEPS REFERENCE POINTS AND NEVER RELATIVE MOVES
---------------------------------------------------
A step is `goto <named point>`, not `jog z -50`. That is the whole reason a
pipeline is reproducible.

A jog is relative to wherever the arm happens to be, so a recorded jog only
replays correctly if every step before it landed exactly where it did during
recording -- and they do not, because the controller tracks to a tolerance
and because a failed step leaves the arm somewhere else entirely. A sequence
of relative moves drifts, and it drifts silently: every step "succeeds" and
the tip ends up somewhere nobody taught.

Absolute joint configurations cannot drift. Each step is a place the robot
was actually driven to and a person actually looked at. Jogging is still how
you GET to those places -- it just is not what gets recorded.

That is also why there is no `jog` step kind to add by hand. It is not an
omission to be filled in later; allowing one would make every pipeline
containing it untrustworthy, and the trustworthiness is the point.
"""
import datetime

# What a step may be, and what its argument means. Adding a kind means adding
# it here, giving it a validator, and teaching Pendant how to execute it --
# in that order, so a file can never name a step the pendant cannot run.
STEP_KINDS = ('goto', 'grip', 'wait')

# Bounds on the numeric steps. Both are refused rather than clamped, for the
# same reason jog distances are (see MAX_JOG_MM in teach_points): a clamped
# value does something other than what the file says, and the file is the
# thing a person reads to find out what the robot will do.
#
# 0.8 rad is the 2F-85 closed against itself; there is nothing above it to
# command. A `wait` longer than a minute is a typo for seconds, not a plan.
#
# The FLOOR is not zero, and that is the interesting one. The knuckle
# joint's lower limit is 0.0, and a knuckle left at rest on it stops
# responding to gripper commands for the rest of the simulator run --
# measured, dead after a 10s dwell at 0.000, fine after 300s at 0.020.
# The write-up is beside GRIPPER_LOWER_LIMIT in bartender_open/arm.py.
# A pipeline is a file a person reads to find out what the robot will
# do, so `grip: 0` is refused here rather than quietly turned into 0.02.
MIN_GRIP_RAD = 0.02
MAX_GRIP_RAD = 0.8
MIN_WAIT_S = 0.0
MAX_WAIT_S = 60.0


class PipelineError(Exception):
    """A pipeline is malformed, or a step in it cannot be understood."""


class Step:
    """One instruction in a pipeline.

    `kind` is one of STEP_KINDS and `arg` is its argument: a point name for
    `goto`, radians for `grip`, seconds for `wait`. The pair is validated on
    construction, so a Step that exists is a Step the pendant can run.
    """

    def __init__(self, kind, arg, note='', arm=None):
        if kind not in STEP_KINDS:
            raise PipelineError(
                f'no step kind {kind!r}; known: {", ".join(STEP_KINDS)}')
        self.kind = kind
        self.note = str(note or '')
        # WHICH ARM, for the steps that cannot work it out themselves.
        #
        # `goto` never needs this: a point carries the joint names it was
        # taught with, so there is exactly one arm it can mean. `grip` has
        # nothing to go on -- 0.25 rad is a perfectly good command to either
        # gripper -- so the arm is recorded with the step. Without it a
        # pipeline would close whichever gripper the operator happened to
        # have selected when they pressed run, which is a two-armed version
        # of the bug that taught points already avoid.
        if arm is not None and (not isinstance(arm, str) or not arm.strip()):
            raise PipelineError(
                f'a step\'s arm must be a name like "a" or "b", got {arm!r}')
        self.arm = arm.strip() if isinstance(arm, str) else None
        self.arg = self._checked(kind, arg)

    @staticmethod
    def _checked(kind, arg):
        if kind == 'goto':
            name = str(arg or '').strip()
            if not name:
                raise PipelineError('a goto step needs a point name')
            return name
        # bool is an int in Python and `grip: true` is a plausible thing to
        # type into a YAML file; it would otherwise arrive here as 1.0 rad.
        if isinstance(arg, bool) or not isinstance(arg, (int, float)):
            raise PipelineError(
                f'a {kind} step needs a number, got {arg!r}')
        value = float(arg)
        lo, hi, unit = ((MIN_GRIP_RAD, MAX_GRIP_RAD, 'rad')
                        if kind == 'grip'
                        else (MIN_WAIT_S, MAX_WAIT_S, 's'))
        if not lo <= value <= hi:
            raise PipelineError(
                f'{kind} {value:g}{unit} is outside '
                f'{lo:g}..{hi:g}{unit}')
        return value

    def __eq__(self, other):
        return (isinstance(other, Step) and self.kind == other.kind
                and self.arg == other.arg and self.note == other.note
                and self.arm == other.arm)

    def __repr__(self):
        return f'Step({self.kind!r}, {self.arg!r})'

    def describe(self):
        """One line, in the same words the pendant takes as a command."""
        if self.kind == 'goto':
            body = f'goto {self.arg}'
        elif self.kind == 'grip':
            body = f'grip {self.arg:.3f}'
            if self.arm:
                body += f' (arm {self.arm})'
        else:
            body = f'wait {self.arg:g}s'
        return f'{body}  -- {self.note}' if self.note else body

    def to_dict(self):
        arg = self.arg if self.kind == 'goto' else round(float(self.arg), 6)
        d = {self.kind: arg}
        if self.arm:
            d['arm'] = self.arm
        if self.note:
            d['note'] = self.note
        return d

    @classmethod
    def from_dict(cls, d, where='<memory>'):
        if not isinstance(d, dict):
            raise PipelineError(
                f'{where}: step is {type(d).__name__}, expected a mapping '
                f'like {{goto: some_point}}')
        kinds = [k for k in d if k in STEP_KINDS]
        if len(kinds) != 1:
            raise PipelineError(
                f'{where}: a step must name exactly one of '
                f'{", ".join(STEP_KINDS)}; got {sorted(d) or "nothing"}')
        unknown = set(d) - {kinds[0], 'note', 'arm'}
        if unknown:
            raise PipelineError(
                f'{where}: step has unexpected key(s) '
                f'{", ".join(sorted(unknown))}')
        return cls(kinds[0], d[kinds[0]], d.get('note', ''), d.get('arm'))


class Pipeline:
    """A named, ordered list of steps."""

    def __init__(self, name, steps=None, note='', recorded=None):
        self.name = name
        self.steps = list(steps or [])
        self.note = note or ''
        self.recorded = recorded or datetime.datetime.now().isoformat(
            timespec='seconds')

    def __len__(self):
        return len(self.steps)

    def append(self, step):
        self.steps.append(step)
        return step

    def point_names(self):
        """Every point this pipeline drives to, in order, with repeats."""
        return [s.arg for s in self.steps if s.kind == 'goto']

    def missing_points(self, store):
        """Names this pipeline needs that `store` does not have.

        Checked when a pipeline is RUN or SHOWN, never when it is loaded. A
        pipeline may legitimately be written before its points are taught, and
        refusing to load the file over that would take the whole point store
        down -- including for the action servers, which do not care about
        pipelines at all.
        """
        seen, missing = set(), []
        for name in self.point_names():
            if name not in store and name not in seen:
                missing.append(name)
            seen.add(name)
        return missing

    def describe(self):
        """Give a heading line for a listing."""
        line = f'{self.name:<24} {len(self.steps)} step(s)'
        if self.note:
            line += f'\n{"":<24}  {self.note}'
        return line

    def to_dict(self):
        d = {'steps': [s.to_dict() for s in self.steps],
             'recorded': self.recorded}
        if self.note:
            d['note'] = self.note
        return d

    @classmethod
    def from_dict(cls, name, d, path='<memory>'):
        if not isinstance(d, dict):
            raise PipelineError(
                f"{path}: pipeline '{name}' is {type(d).__name__}, expected a "
                f'mapping with a `steps:` list')
        steps = d.get('steps')
        if steps is None:
            steps = []
        if not isinstance(steps, list):
            raise PipelineError(
                f"{path}: pipeline '{name}' has `steps:` of type "
                f'{type(steps).__name__}, expected a list')
        out = []
        for index, raw in enumerate(steps, 1):
            where = f"{path}: pipeline '{name}' step {index}"
            try:
                out.append(Step.from_dict(raw, where))
            except PipelineError as exc:
                # Prefixed with the position here rather than left to the
                # Step, because the bound checks in _checked() cannot know
                # where they were called from -- and "grip 9rad is outside
                # 0..0.8rad" is useless in a file with thirty steps.
                message = str(exc)
                raise PipelineError(
                    message if message.startswith(where)
                    else f'{where}: {message}') from None
        return cls(name, out, d.get('note', ''), d.get('recorded'))
