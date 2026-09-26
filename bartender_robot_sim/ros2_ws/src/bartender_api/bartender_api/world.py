"""GET /world: what is there, and who can reach it.

Static geometry from bartender_open.layout; live occupancy and pose from
whatever is actually subscribed to /world/bar_world/dynamic_pose/info (see
server.py) -- the same ground-truth topic open_action_server reads. See
docs/CONTROL_API.md, "1. World".
"""
from dataclasses import asdict, dataclass, fields

from bartender_open import layout as L

from .drink.label import Label
from .perception import Observation, unknown
from .reach import arms_within_reach

# A station's world (x, y) is decided once, in layout.py; its Gazebo MODEL
# NAME is decided separately, by whatever generated it (make_beer_and_opener,
# make_bottle_stands, bar_world.sdf's own <include> blocks), and the two
# vocabularies do not match -- 'whiskey' the station is 'jack_daniels_bottle'
# the model. This is the one place that translation has to happen for
# /world to answer "is something there" from live poses.
STATION_MODEL = {
    'whiskey': 'jack_daniels_bottle',
    'cola': 'cola_bottle',
    'beer': 'beer_bottle',
    'glass': 'serving_glass',
    'opener': 'bottle_opener',
}


@dataclass
class Arm:
    """An arm's base pose and what it can do, as /world reports it."""
    id: str
    origin: list
    yaw: float
    serves_world_y: list
    skills: list


@dataclass
class Counter:
    """The bar counter box; `top_z` is the surface bottles stand on."""
    centre: list
    size: list
    top_z: float


@dataclass
class GroundTruthPose:
    """A pose read from the simulator, so exact by definition."""
    xyz: list
    source: str = 'sim_ground_truth'
    confidence: float = 1.0


@dataclass
class GroundTruth:
    """Station occupancy from the sim pose topic; no pose means empty."""
    occupied: bool
    pose: GroundTruthPose | None


@dataclass
class Comparison(Observation):
    """A camera observation beside the sim truth, for measuring error_mm."""
    ground_truth_xy: list | None = None
    error_mm: float | None = None


@dataclass
class Station:
    """A place on the bar; `live` says what is there and how that is known."""
    id: str
    kind: str
    xy: list
    reachable_by: list
    live: GroundTruth | Observation
    label: Label | None = None


@dataclass
class World:
    """The GET /world document."""
    frame: str
    perception: str
    counter: Counter
    arms: list
    stations: list

    def to_json(self):
        """Serialise, flattening each station's `live` beside its geometry."""
        doc = asdict(self)
        for station in doc['stations']:
            station.update(station.pop('live'))
            if station['label'] is None:
                del station['label']
        return doc


ARMS = (
    Arm('a', list(L.ARM_A_ORIGIN), L.ARM_A_YAW, list(L.APPROACH_WINDOW),
        ['pour', 'hold']),
    Arm('b', list(L.ARM_B_ORIGIN), L.ARM_B_YAW, list(L.APPROACH_WINDOW),
        ['open', 'hold']),
)


def _kind_of(name):
    if name == 'glass':
        return 'vessel'
    if name == 'opener':
        return 'tool'
    return 'bottle'


def _reachable_by(kind, xy):
    """Arms that can reach a station, by the check its kind actually needs.

    A 'bottle' station is picked up by a fixed side-grasp off the line, so
    layout.servicing_arms (APPROACH_WINDOW) is the right question. Nothing
    grips the glass (a bottle is tilted over it) or descends onto the
    opener the same way (straight down, not a side approach), so those use
    plain radial reach instead -- see reach.py for why this split exists
    and what happens if the two are conflated (reachable_by came back []
    for both, which a real bottle successfully poured into every day).
    """
    if kind == 'bottle':
        return L.servicing_arms(xy)
    return arms_within_reach(xy)


def _slot_id(xy):
    """Name an empty line slot the way layout.slot_y's own index would.

    slot_y(index) = index * SLOT_PITCH, so this is just that inverted --
    kept in step with BOTTLE_SLOTS rather than re-deriving a naming scheme,
    so a re-pitched line (Phase D option 3) renames itself for free.
    """
    index = round(xy[1] / L.SLOT_PITCH)
    return f'slot_{index:+d}'


MODES = ('ground_truth', 'camera', 'compare')


def _ground_truth(live):
    return GroundTruth(live is not None,
                       None if live is None else GroundTruthPose(list(live)))


def _compare(seen, live):
    result = Comparison(
        **{f.name: getattr(seen, f.name) for f in fields(seen)})
    if live is not None:
        result.ground_truth_xy = list(live[:2])
        if seen.pose is not None:
            cam = seen.pose.xyz
            result.error_mm = round(
                1000.0 * ((cam[0] - live[0]) ** 2
                          + (cam[1] - live[1]) ** 2) ** 0.5, 1)
    return result


def _live(name, kind, live, observe, mode):
    if mode == 'ground_truth' or (mode == 'compare' and kind != 'bottle'):
        return _ground_truth(live)
    if kind != 'bottle':
        # Unknown, never "unoccupied", so camera mode has no silent fallback.
        return unknown('not observed by any camera')
    seen = observe(name)
    return _compare(seen, live) if mode == 'compare' else seen


def build(pose_lookup, observe=None, mode='ground_truth', see_label=None):
    """Assemble /world from injected lookups, with bottle poses per `mode`."""
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}, not {mode!r}')
    if mode != 'ground_truth' and observe is None:
        raise ValueError(f'mode {mode!r} needs an observe callable')
    stations = []
    for name in sorted(L.STATIONS):
        xy = L.STATIONS[name]
        kind = _kind_of(name)
        live = _live(name, kind, pose_lookup(STATION_MODEL.get(name, name)),
                     observe, mode)
        stations.append(Station(
            name, kind, list(xy), _reachable_by(kind, xy), live,
            see_label(name, live.occupied)
            if see_label and kind == 'bottle' else None))
    for xy in L.free_slots():
        stations.append(Station(_slot_id(xy), 'empty_slot', list(xy),
                                L.servicing_arms(xy),
                                GroundTruth(False, None)))
    return World('world', mode,
                 Counter(list(L.COUNTER_CENTRE), list(L.COUNTER_SIZE),
                         L.COUNTER_Z),
                 list(ARMS), stations)
