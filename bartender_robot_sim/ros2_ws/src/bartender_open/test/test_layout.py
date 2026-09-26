"""Tests for the two-armed opening layout.

What these protect is agreement between things that have no runtime link.

The description places arm B relative to arm A. The world file places arm B's
pedestal, the beer, the opener and the cap in world coordinates.
bartender_open/layout.py states all of it again in each arm's own frame, and
restates the beer, cap and opener dimensions that make_beer_and_opener.py
decides. Nothing checks any of that when the simulator runs. A pedestal 50mm
from where the arm actually is looks fine; an opener aimed at a cap that is
20mm from where the layout says it is looks like bad luck with the physics.

The second thing is reach. Every pose here is inside a UR5e's 850mm, and the
beer is deliberately about the same distance from both arms because both have
to work on it at once. That is easy to lose by nudging a station 100mm to make
room for something.
"""
import importlib.util
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo',
                                'scripts'))

from bartender_open import layout as L                 # noqa: E402
import make_beer_and_opener as gen                     # noqa: E402

WORLD = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'worlds',
                     'bar_world.sdf')
XACRO = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_description', 'urdf',
                     'bartender.urdf.xacro')
MODELS = os.path.join(REPO, 'models')

UR5E_REACH = 0.85
COUNTER_Z = 0.9
POUR = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_pour',
                    'bartender_pour', 'pour_action_server.py')


def _load_render_script():
    """Import render_bartender_urdf.py by path, or None if it is not here.

    It lives in another package's scripts/ directory, which is not on any
    import path: bartender_description is CMake and installs no module.
    """
    path = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_description',
                        'scripts', 'render_bartender_urdf.py')
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location('_render_for_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def world_poses():
    root = ET.parse(WORLD).getroot()
    return {inc.find('name').text:
            [float(v) for v in inc.find('pose').text.split()]
            for inc in root.iter('include')}


# -- the two frames agree ---------------------------------------------------

def test_arm_b_origin_matches_the_description():
    """Compare layout's world position for arm B with the xacro's.

    These are the same point written two ways: the xacro attaches arm B to
    the same `world` link arm A hangs off, so its xyz is relative to arm A's
    base, while everything in the world file and in layout is absolute. Get
    one of them wrong and the arm is simply somewhere else than the pedestal
    under it, and than every pose computed for it.
    """
    text = open(XACRO).read()
    xyz = [float(v) for v in
           text.split('name="arm_b_xyz" value="')[1].split('"')[0].split()]
    assert xyz == pytest.approx(list(L.ARM_B_IN_A), abs=1e-9)

    yaw = float(text.split('name="arm_b_rpy" value="')[1]
                .split('"')[0].split()[2])
    assert yaw == pytest.approx(L.ARM_B_YAW, abs=1e-9)


def test_neither_arm_stands_on_a_pedestal_any_more():
    """Both arms are bolted to the bar top, so nothing holds arm B up.

    The pedestal was scenery and a planning-scene box for as long as the
    counter was too small to carry arm B. Leaving the model in the world
    after widening the counter would put a 0.9m block through the worktop,
    and leaving the box in bartender_open's scene would make a volume the
    arm legitimately occupies unplannable.
    """
    assert 'arm_b_pedestal' not in world_poses()
    server = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_open',
                          'bartender_open', 'open_action_server.py')
    assert "_obj('arm_b_pedestal'" not in open(server).read()


def test_the_counter_matches_the_world():
    """Compare layout's copy of the bar top with the model and its pose."""
    pose = world_poses()['bar_counter']
    assert pose[:2] == pytest.approx(list(L.COUNTER_CENTRE), abs=1e-9)
    assert pose[2] == pytest.approx(COUNTER_Z, abs=1e-9)

    model = os.path.join(MODELS, 'bar_counter', 'model.sdf')
    box = ET.parse(model).getroot().find('.//collision/geometry/box/size')
    size = [float(v) for v in box.text.split()]
    assert size[:2] == pytest.approx(list(L.COUNTER_SIZE), abs=1e-9)
    assert size[2] == pytest.approx(COUNTER_Z, abs=1e-9)


def test_both_arms_stand_on_the_bar_top():
    """Not beside it, and not hanging off the edge of it.

    0.20 of margin is the arm's own footprint plus room for the base ring;
    what this really catches is a counter resized without moving the arms,
    or arms moved without resizing the counter.
    """
    for name, origin in (('A', L.ARM_A_ORIGIN), ('B', L.ARM_B_ORIGIN)):
        for axis, centre, size in ((0, L.COUNTER_CENTRE[0], L.COUNTER_SIZE[0]),
                                   (1, L.COUNTER_CENTRE[1], L.COUNTER_SIZE[1])):
            assert abs(origin[axis] - centre) < size / 2.0 - 0.20, (
                f'arm {name} is off the edge of the bar top on axis {axis}')


def test_the_arms_face_each_other_across_the_line():
    """The arrangement every station's frame conversion assumes.

    Arm B is turned through pi, which is what puts the shared beer at
    POSITIVE y in both arms' frames. Standing them parallel instead is the
    thing that looks right and measures wrong; see APPROACH_WINDOW.
    """
    assert L.ARM_B_ORIGIN[2] == pytest.approx(L.ARM_A_ORIGIN[2], abs=1e-9)
    assert abs(L.ARM_B_YAW - L.ARM_A_YAW) == pytest.approx(math.pi, abs=1e-9)
    # Each arm's +x points at the line, from its own side of it.
    for origin, yaw in ((L.ARM_A_ORIGIN, L.ARM_A_YAW),
                        (L.ARM_B_ORIGIN, L.ARM_B_YAW)):
        _x, _y, _z = L.to_arm((L.BOTTLE_LINE_X, origin[1], L.COUNTER_Z),
                              origin, yaw)
        assert _x > 0.0, 'the bottle line is behind this arm'
    # Far enough apart that neither folds into the other, close enough that
    # the middle slot is inside both.
    apart = math.dist(L.ARM_A_ORIGIN[:2], L.ARM_B_ORIGIN[:2])
    assert 0.8 < apart < 2.0 * L.reach(L.station_in_arm('beer', 'a'))


def test_both_arm_bases_sit_at_counter_height():
    """The one coincidence the whole layout leans on.

    Both arms are mounted level with the counter top, so a z in either arm's
    frame means "height above the counter" and the two agree. Every height in
    layout.py is written that way -- grasp heights measured up a bottle from
    its base, a seated bell measured down from a cap -- and none of them says
    which arm it is for, because it does not have to.
    """
    assert L.ARM_A_ORIGIN[2] == COUNTER_Z
    assert L.ARM_B_ORIGIN[2] == COUNTER_Z


@pytest.mark.parametrize('station,model', [
    ('beer', 'beer_bottle'),
    ('opener', 'bottle_opener'),
    ('whiskey', 'jack_daniels_bottle'),
    ('cola', 'cola_bottle'),
    ('glass', 'serving_glass'),
])
def test_station_matches_the_world(station, model):
    pose = world_poses()[model]
    assert pose[:2] == pytest.approx(list(L.STATIONS[station]), abs=1e-9)


def test_every_station_is_a_slot_or_an_arms_own_spot():
    """Nothing has a position of its own outside the three that are decided.

    The value of the redesign is that a position is a slot index or one of
    the two working stations, not a surveyed pair of numbers. A station that
    is neither has been placed by hand and will be the one nobody moves when
    the rest of the bar does.
    """
    slots = {y for y, _name in L.BOTTLE_SLOTS}
    own = {L.STATION_GLASS, L.STATION_OPENER}
    for name, xy in L.STATIONS.items():
        if xy in own:
            continue
        assert xy[0] == L.BOTTLE_LINE_X, f'{name} is on no row'
        assert xy[1] in slots, f'{name} is on the line but not in a slot'


def test_the_empty_slots_are_really_empty():
    """The room for expansion has to be room, not an unlisted bottle."""
    placed = {tuple(p[:2]) for p in world_poses().values()}
    for x, y in L.free_slots():
        assert (x, y) not in placed, (
            f'slot ({x}, {y}) is listed as free but the world file puts '
            f'something there')
    assert len(L.free_slots()) >= 2


def test_the_slots_are_evenly_spaced_and_centred():
    ys = [y for y, _ in L.BOTTLE_SLOTS]
    assert ys == sorted(ys)
    gaps = [b - a for a, b in zip(ys, ys[1:])]
    assert gaps == pytest.approx([L.SLOT_PITCH] * len(gaps), abs=1e-9)
    assert sum(ys) == pytest.approx(0.0, abs=1e-9)
    assert L.slot_y(0) == pytest.approx(L.STATIONS['beer'][1], abs=1e-9)


def test_every_slot_can_be_serviced_by_an_arm():
    """A slot no arm can take a bottle off is tabletop, not a station.

    This is the test that keeps the line honest about its own length. The
    window it checks against was measured, not assumed; see
    layout.APPROACH_WINDOW.
    """
    for y, name in L.BOTTLE_SLOTS:
        arms = L.servicing_arms((L.BOTTLE_LINE_X, y))
        assert arms, (
            f'the slot at y={y} ({name or "free"}) is outside both arms\' '
            f'approach windows')


def test_the_beer_can_be_serviced_by_both_arms():
    """It is the one slot that has to be, because both work it at once."""
    assert L.servicing_arms(L.STATIONS['beer']) == ['a', 'b']


def test_the_pour_sweep_misses_every_standing_bottle():
    """The collision that decides where the glass goes.

    A bottle is poured by tilting it about its grip point over the glass.
    At POUR_TILT it lies nearly flat with its base about 0.30 behind the
    glass and 0.18 above the counter -- below the 0.3055 top of anything
    standing in the line. So the pour sweeps back across the line, and what
    keeps it off the bottles is the glass being offset in y from every slot.
    What has to fit in the gap is the whiskey's 0.0546 envelope plus the
    widest standing one's, which is the whiskey's again.
    """
    assert L.pour_sweep_clearance() > 0.0546 + 0.0546 + 0.05


def test_the_beer_stand_is_under_the_beer():
    """A stand offset from its bottle is a stand the bottle lands on the rim of."""
    assert (world_poses()['beer_stand'][:2]
            == pytest.approx(world_poses()['beer_bottle'][:2], abs=1e-9))


def test_the_holster_is_under_the_opener():
    assert (world_poses()['opener_holster'][:2]
            == pytest.approx(world_poses()['bottle_opener'][:2], abs=1e-9))


def test_the_cap_starts_on_the_bottles_mouth():
    """Not a millimetre lower.

    The cap and the bottle are separate models held together by a
    DetachableJoint. While they are joined they are one skeleton and overlap
    is survivable; the instant the joint lets go they are two bodies, and two
    bodies interpenetrating is how the cap gets fired across the room. So it
    sits exactly ON the lip.
    """
    beer = world_poses()['beer_bottle']
    cap = world_poses()['beer_cap']
    assert cap[:2] == pytest.approx(beer[:2], abs=1e-9)
    assert cap[2] == pytest.approx(beer[2] + L.BEER_HEIGHT, abs=1e-6)


def test_the_opener_rests_on_its_rim():
    """Its origin is the bell rim, and the rim sits on the counter.

    The post is shorter than the bell is deep, on purpose, so this is the
    only place it can come to rest. Placing the opener at the post's top
    instead leaves it hanging 14mm in the air at t=0 and it drops.
    """
    pose = world_poses()['bottle_opener']
    assert pose[2] == pytest.approx(COUNTER_Z + L.OPENER_REST_RIM_Z, abs=1e-9)
    assert gen.post_top() < gen.bell_height()


# -- the numbers restated from the generator --------------------------------

@pytest.mark.parametrize('name,mine,theirs', [
    ('BEER_HEIGHT', lambda: L.BEER_HEIGHT, lambda: gen.BEER_HEIGHT),
    ('BEER_BODY_RADIUS', lambda: L.BEER_BODY_RADIUS,
     lambda: gen.BEER_BODY_RADIUS),
    ('BEER_GRIP_BAND', lambda: L.BEER_GRIP_BAND, lambda: gen.BEER_GRIP_BAND),
    # Rounded to a tenth of a millimetre where the generator measures it off
    # the mesh to full precision. That is deliberate -- it is the width the
    # gripper is told to close on, and a gripper that cares about the seventh
    # decimal place of a bottle is not a gripper -- so this one is checked to
    # the precision it is written at.
    ('BEER_GRIP_RADIUS', lambda: L.BEER_GRIP_RADIUS,
     lambda: round(gen.beer_grip_radius(), 4)),
    ('CAP_HEIGHT', lambda: L.CAP_HEIGHT, lambda: gen.CAP_HEIGHT),
    ('CAP_RADIUS', lambda: L.CAP_RADIUS, lambda: gen.CAP_RADIUS),
    ('BELL_HEIGHT', lambda: L.BELL_HEIGHT, lambda: gen.bell_height()),
    ('BELL_CLEARANCE', lambda: L.BELL_CLEARANCE, lambda: gen.BELL_CLEARANCE),
    ('BELL_CAPTURE', lambda: L.BELL_CAPTURE, lambda: gen.bell_capture()),
    ('BELL_OUTER_RADIUS', lambda: L.BELL_OUTER_RADIUS,
     lambda: gen.bell_outer_radius()),
    ('OPENER_HEIGHT', lambda: L.OPENER_HEIGHT, lambda: gen.opener_height()),
    ('OPENER_GRIP_Z', lambda: L.OPENER_GRIP_Z, lambda: gen.shaft_grip_z()),
    ('OPENER_SHAFT', lambda: L.OPENER_SHAFT, lambda: gen.SHAFT),
    ('HOLSTER_POST_TOP', lambda: L.HOLSTER_POST_TOP, lambda: gen.post_top()),
])
def test_dimension_matches_the_generator(name, mine, theirs):
    """layout.py restates the generator's numbers; they must still be its numbers.

    Restated rather than imported because the generator is a standalone
    script in another package and is not installed as a module. This is what
    stops the copy drifting, and drift here is silent: the opener would be
    aimed a few millimetres wrong and the bell would land on the cap instead
    of over it.
    """
    assert mine() == pytest.approx(theirs(), abs=1e-9)


def test_the_beer_grip_band_is_the_straight_part_of_the_neck():
    """The band the pads close on must be the widest thing near it.

    If the shoulder below it is not wider, there is no step, and without the
    step this is just another smooth cylinder between two flat pads -- which
    is the grip that threw this bottle 3.8 metres.
    """
    bands = {(z0, z1): r for z0, z1, r in gen.beer_bands()}
    grip = bands[L.BEER_GRIP_BAND]
    below = [r for (z0, z1), r in bands.items() if z1 <= L.BEER_GRIP_BAND[0]]
    above = [r for (z0, z1), r in bands.items() if z0 >= L.BEER_GRIP_BAND[1]]
    assert min(below) > grip + 0.005, 'no step under the grip band'
    assert max(above) < grip, 'the neck must narrow above the grip'


def test_the_pads_clear_the_shoulder_below_the_grip_band():
    """The pad is wider than the band, so it must overhang the free end.

    Above 0.200 the bottle tapers away and an overhanging pad never touches.
    Below 0.170 is the shoulder step, and a pad reaching onto it grips that
    instead: the fingers stall at 41.6mm rather than the neck's 38.7mm -- a
    real grasp, but a different one from run to run, which no grip check can
    accept. The clearance has to beat the ~3mm a Cartesian move lands within.
    """
    pad_low = L.BEER_GRASP_HEIGHT - L.PAD_HALF_WIDTH
    assert pad_low > L.BEER_GRIP_BAND[0] + 0.003
    # The pad really is wider than the band -- if that stops being true, the
    # overhang reasoning above is no longer what is keeping this safe.
    assert 2 * L.PAD_HALF_WIDTH > L.BEER_GRIP_BAND[1] - L.BEER_GRIP_BAND[0]


def test_the_pad_still_contacts_most_of_the_neck_band():
    """Moving the grip up the neck trades contact for shoulder clearance.

    It can only be traded so far: the band is all the contact this bottle
    has, and the beer is the grasp that already fails most.
    """
    contact = min(L.BEER_GRASP_HEIGHT + L.PAD_HALF_WIDTH, L.BEER_GRIP_BAND[1])
    contact -= max(L.BEER_GRASP_HEIGHT - L.PAD_HALF_WIDTH, L.BEER_GRIP_BAND[0])
    assert contact > 0.024


def test_the_pad_width_agrees_with_the_description():
    """PAD_HALF_WIDTH is a copy, and copies drift.

    render_bartender_urdf.py derives the pad width from a table that carries
    its own copy of BEER_GRASP_HEIGHT, so a change to either file that is not
    made in both leaves the arm planning against a gripper it does not have.
    Loaded by path because bartender_description is a CMake package with no
    importable module; skipped rather than failed when it is not beside us,
    so an installed-only test run does not report a phantom failure.
    """
    render = _load_render_script()
    if render is None:
        pytest.skip('bartender_description sources are not in this tree')
    assert render.PAD_HALF_WIDTH == pytest.approx(L.PAD_HALF_WIDTH, abs=1e-9)
    beer = [row for row in render.GRIP_BANDS if row[0] == 'beer'][0]
    assert beer[3] == pytest.approx(L.BEER_GRASP_HEIGHT, abs=1e-9)
    assert (beer[1], beer[2]) == pytest.approx(L.BEER_GRIP_BAND, abs=1e-9)
    # ...and the width really is the limit that table implies, not a number
    # someone picked and left behind when a bottle moved.
    assert render.PAD_HALF_WIDTH == pytest.approx(
        render.pad_half_width_limit(), abs=1e-9)


# -- the geometry of the press ----------------------------------------------

def test_a_seated_bell_is_deeper_than_the_test_for_it():
    assert L.SEAT_DEPTH_MIN < L.BELL_HEIGHT
    # ...and past the chamfer, into the straight bore, which is what
    # distinguishes "over the cap" from "resting on it".
    assert L.SEAT_DEPTH_MIN > gen.BELL_LEAD_H


def test_the_bell_clears_the_cap_and_funnels_it():
    assert L.BELL_CLEARANCE > 0.0
    assert L.BELL_CAPTURE > L.BELL_CLEARANCE
    assert L.SEAT_OFFSET_MAX == pytest.approx(L.BELL_CAPTURE, abs=1e-9)


def test_the_bell_clears_the_pads_holding_the_bottle():
    """Arm B's bell must not reach down to arm A's fingers.

    Both are on the bottle's axis, so there is no lateral clearance at all
    between them -- the bell's 23.6mm outer radius swallows fingers closed to
    a 38.7mm neck. All the clearance there is, is height.
    """
    pad_top = L.BEER_GRIP_BAND[1] + L.HOLD_LIFT
    assert L.seated_rim_z() > pad_top + 0.02


def test_the_lift_clears_the_stand():
    assert L.HOLD_LIFT > L.STAND_HEIGHT


def test_the_transit_clears_a_capped_bottle_in_its_stand():
    assert L.APPROACH_Z > L.cap_top_above_base() + 0.10


def test_the_keepout_is_below_the_transit_and_above_the_bottle():
    """Its height is what makes it usable rather than merely safe.

    The keep-out has to stop a plan dipping towards the bottle while still
    allowing the one pose every approach ends at, which is directly above it.
    That only works if the volume stops short of the transit height.
    """
    assert L.BEER_KEEPOUT_HEIGHT > L.cap_top_above_base()
    assert L.BEER_KEEPOUT_HEIGHT < L.APPROACH_Z - 0.05
    assert L.BEER_KEEPOUT_RADIUS > L.BEER_BODY_RADIUS


def test_the_shift_limit_allows_the_push_and_catches_a_knock():
    """It must be well over the give under a press and well under a knock.

    The give is the arm's own compliance plus a round neck settling in the
    pads; measured between 3.7 and 19.5mm on presses that seated the bell.
    """
    assert L.BOTTLE_SHIFT_MAX > 4 * L.PRESS_TRAVEL
    assert L.BOTTLE_SHIFT_MAX < L.HOLD_LIFT


def test_a_bottle_left_standing_fails_the_held_check():
    """The check a standing bottle cannot fake.

    Push down on a bottle resting in its well and it does not move either --
    the counter takes the load. So the gate cannot be about movement alone;
    it has to ask whether the bottle is off the counter, which is only true
    if the other arm is holding it.
    """
    assert L.MIN_HELD_CLEARANCE > L.STAND_HEIGHT
    assert L.MIN_HELD_CLEARANCE < L.HOLD_LIFT


def test_a_cap_still_on_the_bottle_does_not_count_as_off():
    """CAP_FREE_MIN has to exceed how far a cap can be and still be sitting there.

    It is a disc resting on the lip, so "still on" is not exactly zero: it
    was measured settling 13mm away, leaning against the neck, after a detach
    with no flick. The threshold has to be above that.
    """
    assert L.CAP_FREE_MIN > 0.020


# -- reach and the counter --------------------------------------------------

@pytest.mark.parametrize('station', sorted(L.STATIONS))
def test_station_is_on_the_counter(station):
    x, y = L.STATIONS[station]
    assert (abs(x - L.COUNTER_CENTRE[0])
            < L.COUNTER_SIZE[0] / 2.0 - L.BEER_STAND_RADIUS)
    assert (abs(y - L.COUNTER_CENTRE[1])
            < L.COUNTER_SIZE[1] / 2.0 - L.BEER_STAND_RADIUS)


@pytest.mark.parametrize('x,y', L.free_slots())
def test_a_free_slot_is_on_the_counter_too(x, y):
    """An expansion slot nobody can put a bottle in is not expansion room."""
    assert (abs(y - L.COUNTER_CENTRE[1])
            < L.COUNTER_SIZE[1] / 2.0 - L.BEER_STAND_RADIUS)


@pytest.mark.parametrize('x,y', L.free_slots())
def test_a_free_slot_is_reachable_by_at_least_one_arm(x, y):
    """Same point: a slot has to be servable or it is just tabletop."""
    best = min(L.reach(L.to_arm((x, y, COUNTER_Z), origin, yaw))
               for origin, yaw in ((L.ARM_A_ORIGIN, L.ARM_A_YAW),
                                   (L.ARM_B_ORIGIN, L.ARM_B_YAW)))
    assert best < UR5E_REACH * 0.8


def test_the_bar_top_is_bigger_than_everything_standing_on_it():
    """The counter has to contain the whole layout with a working margin.

    Written as an envelope rather than as two numbers so that moving a
    station or adding a slot is what fails this, rather than the first plan
    that tries to reach past the edge.
    """
    xs = [L.ARM_A_ORIGIN[0], L.ARM_B_ORIGIN[0], L.BOTTLE_LINE_X]
    ys = [L.ARM_A_ORIGIN[1], L.ARM_B_ORIGIN[1]] + [y for y, _ in L.BOTTLE_SLOTS]
    for x, y in (L.STATION_GLASS, L.STATION_OPENER):
        xs.append(x)
        ys.append(y)
    for lo, hi, centre, size, axis in (
            (min(xs), max(xs), L.COUNTER_CENTRE[0], L.COUNTER_SIZE[0], 'x'),
            (min(ys), max(ys), L.COUNTER_CENTRE[1], L.COUNTER_SIZE[1], 'y')):
        assert lo - (centre - size / 2.0) > 0.20, f'no room at low {axis}'
        assert (centre + size / 2.0) - hi > 0.20, f'no room at high {axis}'


@pytest.mark.parametrize('station,arm', [('beer', 'a'), ('beer', 'b'),
                                         ('opener', 'b')])
def test_station_is_within_reach(station, arm):
    here = L.station_in_arm(station, arm)
    assert L.reach(here) < UR5E_REACH * 0.8


def test_the_beer_is_about_equally_far_from_both_arms():
    """Both arms work on this bottle at the same time, so neither may be stretched.

    It is the only station that has to satisfy two arms at once, which is
    what fixed it where it is.
    """
    from_a = L.reach(L.station_in_arm('beer', 'a'))
    from_b = L.reach(L.station_in_arm('beer', 'b'))
    assert abs(from_a - from_b) < 0.05


def test_the_working_poses_are_within_reach():
    """Not just the stations: the flange poses actually commanded."""
    held = L.side_grasp_tool0(L.beer_grip_point(L.HOLD_LIFT))
    assert L.reach(held) < UR5E_REACH * 0.8
    over = L.side_grasp_tool0(L.opener_over_cap_grip(L.seated_rim_z()))
    assert L.reach(over) < UR5E_REACH * 0.8
    holster = L.side_grasp_tool0(L.opener_grip_point(L.OPENER_REST_RIM_Z))
    assert L.reach(holster) < UR5E_REACH * 0.8


def test_the_beer_is_clear_of_the_other_stations():
    """Its stand must not overlap the whiskey's, and the arms need room."""
    beer = L.STATIONS['beer']
    for name, xy in L.STATIONS.items():
        if name == 'beer':
            continue
        apart = math.dist(xy, beer)
        assert apart > 0.0740 + L.BEER_STAND_RADIUS + 0.02, (
            f'the beer is only {apart:.3f} from the {name}')


def test_a_carry_to_the_glass_passes_over_the_line_not_through_it():
    """In a line, every carry crosses the line, so it has to go over.

    On the old counter the bottles stood apart and a carry never passed one.
    Here they are a slot-pitch apart with the glass off to the side, and the
    plan-view gap is not enough: the cola's diagonal to the glass passes
    59mm from the standing whiskey, against the 95mm of envelope that would
    have to fit there. What makes it safe is height, so that is what this
    checks -- and it checks the geometry that decides the height rather than
    the constant, so a taller bottle fails it.

    Parsed out of bartender_pour, which owns the number.
    """
    if not os.path.exists(POUR):
        pytest.skip('bartender_pour absent')
    text = open(POUR).read()
    match = re.search(r'^CARRY_MOUTH_Z\s*=\s*([\d.]+)', text, re.M)
    assert match, 'CARRY_MOUTH_Z not found in pour_action_server'
    carry_mouth = float(match.group(1))

    heights = {name: float(h) for name, h in re.findall(
        r"name='(\w+)',\s*\n\s*xy=\([^)]*\),\s*\n\s*height=([\d.]+)",
        text)}
    assert heights, 'could not read the bottle heights'
    for name, height in heights.items():
        base = carry_mouth - height
        assert base > L.COUNTER_TALLEST + 0.02, (
            f'a carried {name} rides with its base at {base:.3f}, which is '
            f'not clear of the {L.COUNTER_TALLEST} tall bottle it passes')


def test_the_glass_is_off_every_slot_so_the_pour_sweep_misses():
    """Height does not save the pour itself: that happens low, over the glass."""
    assert L.pour_sweep_clearance() > 0.0546 + 0.0546 + 0.05


# -- frame conversion -------------------------------------------------------

@pytest.mark.parametrize('point', [(0.0, 0.0, 0.9), (0.3, -0.2, 1.1),
                                   (-0.5, 0.4, 0.95)])
@pytest.mark.parametrize('arm', ['a', 'b'])
def test_to_arm_and_to_world_are_inverses(point, arm):
    origin, yaw = ((L.ARM_A_ORIGIN, L.ARM_A_YAW) if arm == 'a'
                   else (L.ARM_B_ORIGIN, L.ARM_B_YAW))
    there = L.to_arm(point, origin, yaw)
    assert L.to_world(there, origin, yaw) == pytest.approx(point, abs=1e-12)


@pytest.mark.parametrize('arm', ['a', 'b'])
def test_each_arm_faces_its_own_work(arm):
    """An arm's +x must point at the bar, not away from it.

    Yaw only decides where joint angles point, but getting it backwards puts
    every side grasp's approach direction 180 degrees out, and the arm tries
    to reach the bar through its own shoulder.
    """
    station = 'glass' if arm == 'a' else 'opener'
    for name in ('beer', station):
        x, _y, _z = L.station_in_arm(name, arm)
        assert x > 0.25, (
            f'the {name} is at x={x:.3f} in arm {arm}\'s frame, which is '
            f'behind or on top of its own shoulder')


def test_the_shared_beer_is_on_both_arms_good_side():
    """The whole reason arm B is turned round.

    APPROACH_WINDOW is a band of POSITIVE y in an arm's own frame. The beer
    is the one station both arms work, so it is the one that has to be in
    both windows at once, and that is only possible with the arms facing.
    """
    for arm in ('a', 'b'):
        _x, y, _z = L.station_in_arm('beer', arm)
        lo, hi = L.APPROACH_WINDOW
        assert lo <= y <= hi, (
            f'the beer is at y={y:.3f} in arm {arm}\'s frame, outside the '
            f'measured approach window {L.APPROACH_WINDOW}')


def test_the_holster_is_closer_in_than_the_line():
    """Arm B descends vertically onto the opener, and that limits reach.

    Measured: a holster 0.67 from arm B's base could not be descended to at
    all -- the arm stopped 346mm short and the goal failed on "arm B could
    not pick up the opener". The line is at 0.53; the holster has to be
    inside that, not beyond it.
    """
    opener = L.reach(L.station_in_arm('opener', 'b'))
    line = L.station_in_arm('beer', 'b')[0]
    assert opener < line, (
        f'the holster is {opener:.3f} from arm B, past the line at {line:.3f}')
    assert opener < 0.60


def test_side_grasp_tool0_sets_the_flange_back_from_the_pads():
    grip = (0.4, 0.1, 0.2)
    flange = L.side_grasp_tool0(grip)
    assert flange[1:] == pytest.approx(grip[1:], abs=1e-12)
    assert grip[0] - flange[0] == pytest.approx(L.GRIP_AHEAD_OF_TOOL0, abs=1e-12)


# -- crossing the counter with the opener ------------------------------------

def test_the_carried_opener_clears_the_tallest_bottle():
    """The height that matters is the bell rim's, not the flange's.

    The opener hangs OPENER_GRIP_Z below the pads, so a flange height that
    looks generous is not. At the old 0.40 the rim sat 17.5mm over the cola's
    spout and 22.5mm over the whiskey's, and it hit.
    """
    rim = L.OPENER_TRANSIT_Z - L.OPENER_GRIP_Z
    assert rim - L.COUNTER_TALLEST == pytest.approx(L.TRANSIT_CLEARANCE)
    assert rim > L.COUNTER_TALLEST + 0.05


def test_the_tallest_bottle_really_is_the_tallest():
    """COUNTER_TALLEST is a copy of a number that lives in the models.

    If a taller bottle is added, or one grows a spout, this is the line that
    is supposed to fail rather than the opener being the thing that finds out.
    """
    tops = {}
    for name in ('jack_daniels_bottle', 'cola_bottle', 'beer_bottle'):
        path = os.path.join(MODELS, name, 'model.sdf')
        if not os.path.exists(path):
            continue
        top = 0.0
        for col in ET.parse(path).getroot().iter('collision'):
            pose = col.find('pose')
            p = [float(v) for v in pose.text.split()] if pose is not None \
                else [0.0] * 6
            cyl = col.find('geometry/cylinder')
            box = col.find('geometry/box')
            if cyl is not None:
                top = max(top, p[2] + float(cyl.find('length').text) / 2.0)
            elif box is not None:
                size = [float(v) for v in box.find('size').text.split()]
                top = max(top, p[2] + size[2] / 2.0)
        tops[name] = top
    assert tops, 'no bottle models found to check against'
    assert L.COUNTER_TALLEST == pytest.approx(max(tops.values()), abs=1e-6), (
        f'COUNTER_TALLEST is {L.COUNTER_TALLEST}, but the models top out at '
        f'{max(tops.values()):.4f} ({max(tops, key=tops.get)})')


def test_the_opener_cruises_higher_than_an_empty_gripper():
    """Carrying something low-hanging needs more height, not the same."""
    assert L.OPENER_TRANSIT_Z > L.APPROACH_Z


def test_arm_b_can_reach_its_stations_at_cruising_height():
    """Raising the transit is only free while it stays inside the arm.

    Going up costs reach, and arm B works at the far end of the counter.
    """
    for station in ('opener', 'beer'):
        x, y, _ = L.station_in_arm(station, 'b')
        tool = L.side_grasp_tool0((x, y, 0.0))
        out = math.hypot(math.hypot(tool[0], tool[1]), L.OPENER_TRANSIT_Z)
        assert out < UR5E_REACH - 0.10, (
            f'arm B would be {out:.3f}m from its base over the {station} '
            f'at cruising height')


# -- bartender_pour states the same bar in arm A's frame ---------------------

def _pour_literals():
    """Read the pour server's station coordinates out of its source text.

    Parsed rather than imported: pour_action_server pulls in rclpy, MoveIt
    messages and the generated action interfaces, so importing it would make
    this only runnable on a built, sourced workspace -- and this is exactly
    the check you want before building. Same reason test_seeded_points.py
    parses it.
    """
    if not os.path.exists(POUR):
        return {}
    text = open(POUR).read()
    found = {}
    match = re.search(r'^GLASS_XY\s*=\s*\(([^)]*)\)', text, re.M)
    if match:
        found['glass'] = tuple(float(v) for v in match.group(1).split(','))
    for name, xy in re.findall(
            r"name='(\w+)',\s*\n\s*xy=\(([^)]*)\)", text):
        found.setdefault(name, tuple(float(v) for v in xy.split(',')))
    return found


def test_the_pour_server_still_names_its_stations():
    """If this fails the parse above has gone stale and the rest is empty."""
    assert set(_pour_literals()) >= {'glass', 'whiskey', 'cola'}


@pytest.mark.parametrize('station', ['glass', 'whiskey', 'cola'])
def test_the_pour_servers_stations_match_the_layout(station):
    """Check one position stated twice, in packages that cannot import each other.

    bartender_pour works entirely in arm A's base frame and has no
    dependency on bartender_open, so its three stations are a second copy of
    numbers this file decides. A copy that drifts does not fail anywhere: the
    arm reaches confidently for a place the bottle is not.
    """
    literals = _pour_literals()
    x, y, _ = L.station_in_arm(station, 'a')
    assert literals[station] == pytest.approx((x, y), abs=1e-9), (
        f"pour_action_server puts the {station} at {literals[station]} in "
        f'base_link; the layout puts it at {(round(x, 4), round(y, 4))}')


def test_the_pour_servers_counter_box_matches_the_bar_top():
    """The planning scene's counter, which is the only one MoveIt sees."""
    if not os.path.exists(POUR):
        pytest.skip('bartender_pour absent')
    text = open(POUR).read()
    match = re.search(r'^COUNTER_BOX\s*=\s*\(\(([^)]*)\),\s*\(([^)]*)\)\)',
                      text, re.M)
    assert match, 'COUNTER_BOX not found in pour_action_server'
    centre = [float(v) for v in match.group(1).split(',')]
    size = [float(v) for v in match.group(2).split(',')]
    # The top is at COUNTER_Z and the box hangs its full 0.9 below, so its
    # centre is half a counter down from the arm bases.
    want = L.to_arm((L.COUNTER_CENTRE[0], L.COUNTER_CENTRE[1],
                     COUNTER_Z - size[2] / 2.0), L.ARM_A_ORIGIN, L.ARM_A_YAW)
    assert centre == pytest.approx(list(want), abs=1e-9)
    assert size[:2] == pytest.approx(list(L.COUNTER_SIZE), abs=1e-9)
