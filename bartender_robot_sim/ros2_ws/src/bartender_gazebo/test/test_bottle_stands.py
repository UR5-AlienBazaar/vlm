"""Tests for the bottle stands.

Two things are being protected here.

The first is the geometry: a well that has a gap in its wall, or whose wall is
somewhere other than where the clearance says, does not announce itself. The
bottle just goes over in one run out of some number and it looks like the old
fault.

The second is the coupling that the design exists to avoid. Bottle.grasp_height
in bartender_pour is measured up from the bottle's base and used directly as a
base_link z, so the instant anything puts a floor under a bottle, every grasp
in the system is quietly low. test_no_floor is the guard on that, and it is the
most important test in this file.
"""
import importlib.util
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo',
                                'scripts'))

from make_bottle_stands import (                       # noqa: E402
    HOLD_H, LEAD, LEAD_H, SEGMENTS, STANDS, WALL, boxes, config, courses,
    model_sdf, outer_radius, segment_poses, total_height,
)

WORLD = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'worlds',
                     'bar_world.sdf')
MODELS = os.path.join(REPO, 'models')

# Which bottle each stand belongs to. The world places both at their
# bottle's own station, which is the whole point -- a stand offset from its
# bottle is a stand the bottle is lowered onto the rim of.
#
# Deliberately no coordinates. WHERE the stations are is decided in
# bartender_open/layout.py and checked model by model against this same
# world file by that package's test_layout.py; a second copy here would be
# a second place to update, and it was exactly that -- it still named the
# pre-redesign (0.15, 0.15) long after the bottles had moved, and nothing
# noticed because nothing read it. The beer_stand is absent for the same
# reason it always was.
STATIONS = {
    'whiskey_stand': 'jack_daniels_bottle',
    'cola_stand': 'cola_bottle',
}


@pytest.fixture(params=sorted(STANDS))
def stand(request):
    name = request.param
    spec = STANDS[name]
    return name, spec, spec['bottle_radius'] + spec['clearance']


# -- the shape of one well --------------------------------------------------

def test_inner_face_is_where_the_clearance_says(stand):
    """Each box's inner face must be tangent to the nominal circle.

    The clearance is quoted against this radius, so if the boxes sit on it by
    their CENTRES instead the wall is half a wall-thickness too far out
    everywhere and every clearance in the design is 4mm optimistic.
    """
    _, _, inner_r = stand
    for x, y, yaw, _ in segment_poses(inner_r):
        # distance from the origin to the box's inner face along its normal
        centre_r = math.hypot(x, y)
        assert centre_r - WALL / 2.0 == pytest.approx(inner_r, abs=1e-12)
        # the box's normal must point radially, or it is not tangent at all
        assert math.atan2(y, x) % (2 * math.pi) == pytest.approx(
            yaw % (2 * math.pi), abs=1e-12)


def test_the_wall_has_no_gaps(stand):
    """Sweep every bearing and check something is always in the way.

    A ring of boxes laid tangentially leaves a wedge-shaped gap at each joint
    unless the boxes are long enough to overlap. The lengths come from the
    OUTER tangent circle for exactly this reason, and this is the check that
    they still do.
    """
    _, _, inner_r = stand
    probe_r = inner_r + WALL / 2.0      # mid-wall
    for i in range(3600):
        bearing = 2.0 * math.pi * i / 3600.0
        px, py = probe_r * math.cos(bearing), probe_r * math.sin(bearing)
        assert any(
            _inside_box(px, py, x, y, yaw, WALL, length)
            for x, y, yaw, length in segment_poses(inner_r)
        ), f'gap in the wall at {math.degrees(bearing):.1f} degrees'


def _inside_box(px, py, cx, cy, yaw, thickness, length):
    dx, dy = px - cx, py - cy
    local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    return (abs(local_x) <= thickness / 2.0 + 1e-12
            and abs(local_y) <= length / 2.0 + 1e-12)


def test_no_floor(stand):
    """NOTHING may sit under the bottle. See this module's docstring.

    The bottle base has to rest on the counter at the same height it always
    did, because bartender_pour measures every grasp up from it.
    """
    _, _, inner_r = stand
    for (_, _, z), _, (_, _, height) in boxes(inner_r):
        assert z - height / 2.0 >= -1e-12, 'geometry below counter level'
    # and nothing may cross the middle, at any height
    for x, y, _, _ in segment_poses(inner_r):
        assert math.hypot(x, y) - WALL / 2.0 > 0.0


def test_the_bottle_fits_in(stand):
    """At its station the bottle must go in without touching the wall."""
    _, spec, inner_r = stand
    assert inner_r > spec['bottle_radius']
    assert inner_r - spec['bottle_radius'] == pytest.approx(spec['clearance'])


def test_the_bottle_cannot_get_out_sideways(stand):
    """The holding course must be a real wall, not a token lip.

    A bottle nudged off-station travels at most `clearance` before the wall
    stops it. What stops it going OVER the wall instead is that the wall is
    tall compared with that travel.
    """
    _, spec, _ = stand
    assert HOLD_H > 2.0 * spec['clearance']


def test_the_lead_in_is_above_the_holding_course_and_wider(stand):
    """A funnel, not a second wall. Wrong order and it is a trap, not a guide."""
    _, _, inner_r = stand
    (r_lo, z_lo, h_lo), (r_hi, z_hi, h_hi) = courses(inner_r)
    assert r_hi > r_lo
    assert z_hi == pytest.approx(z_lo + h_lo)
    assert r_hi - r_lo == pytest.approx(LEAD)
    assert z_hi + h_hi == pytest.approx(total_height())


def test_segment_count_and_box_count(stand):
    _, _, inner_r = stand
    assert len(list(boxes(inner_r))) == 2 * SEGMENTS


# -- against the bottles the wells are for ----------------------------------

def _bottle_base(model, collision_name):
    """(radius-or-halfwidth, top z) of a bottle's base collision."""
    root = ET.parse(os.path.join(MODELS, model, 'model.sdf')).getroot()
    for col in root.iter('collision'):
        if col.get('name') != collision_name:
            continue
        pose = [float(v) for v in col.find('pose').text.split()]
        cyl = col.find('geometry/cylinder')
        if cyl is not None:
            r = float(cyl.find('radius').text)
            h = float(cyl.find('length').text)
        else:
            sx, _, h = [float(v) for v in
                        col.find('geometry/box/size').text.split()]
            r = sx / 2.0 * math.sqrt(2.0)       # a square sits on its corners
        return r, pose[2] + h / 2.0
    raise AssertionError(f'no collision {collision_name} in {model}')


@pytest.mark.parametrize('stand_name,model,collision', [
    ('whiskey_stand', 'jack_daniels_bottle', 'bottle_body_collision'),
    ('cola_stand', 'cola_bottle', 'cola_base_collision'),
])
def test_bottle_radius_matches_the_model(stand_name, model, collision):
    """The generator's numbers are copies. This is what notices them drifting."""
    radius, _ = _bottle_base(model, collision)
    assert STANDS[stand_name]['bottle_radius'] == pytest.approx(radius, abs=5e-5)


@pytest.mark.parametrize('stand_name,model,collision', [
    ('whiskey_stand', 'jack_daniels_bottle', 'bottle_body_collision'),
    ('cola_stand', 'cola_bottle', 'cola_base_collision'),
])
def test_the_wall_is_shorter_than_the_bottle_section_it_holds(
        stand_name, model, collision):
    """The wall must be shorter than the straight part of the bottle's base.

    A taller one is a wall the bottle perches on rather than sits inside.
    """
    _, top = _bottle_base(model, collision)
    assert total_height() < top


def _render_script():
    """Import render_bartender_urdf.py by path, or None if it is not here.

    bartender_description is a CMake package and installs no python module,
    so there is nothing to import normally.
    """
    path = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_description',
                        'scripts', 'render_bartender_urdf.py')
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location('_render_for_stands', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_wall_is_clear_of_the_gripper():
    """The pads must not reach the rim when they close on the lowest bottle.

    The cola is the tight one: gripped at 0.060, lower than anything else in
    the scene, with a pad reaching PAD_HALF_WIDTH below that. The stand only
    has to stay under what is left.

    PAD_HALF_WIDTH is read from the description rather than copied, because
    it is not a settled number -- it went from 11mm to 15mm to 16.5mm as
    bottles were added -- and a copy here would go stale silently in the one
    direction that matters, the pads reaching further down.
    """
    render = _render_script()
    if render is None:
        pytest.skip('bartender_description sources are not in this tree')
    lowest = min(grasp for _n, _lo, _hi, grasp, _lb, _hb in render.GRIP_BANDS)
    assert total_height() < lowest - render.PAD_HALF_WIDTH


# -- the generated files on disk --------------------------------------------

def test_checked_in_model_matches_the_generator(stand):
    """Regenerating must be a no-op.

    The model.sdf in models/ is generated; this is what catches a hand edit,
    or a stale file after the script has been changed.
    """
    name, spec, _ = stand
    on_disk = open(os.path.join(MODELS, name, 'model.sdf')).read()
    assert on_disk == model_sdf(name, spec)


def test_checked_in_config_matches_the_generator(stand):
    name, spec, _ = stand
    on_disk = open(os.path.join(MODELS, name, 'model.config')).read()
    assert on_disk == config(name, spec)


def test_model_is_static(stand):
    name, _, _ = stand
    root = ET.parse(os.path.join(MODELS, name, 'model.sdf')).getroot()
    assert root.find('model/static').text == 'true'


def test_every_collision_carries_friction(stand):
    """Every wall segment must carry friction.

    Unnamed collisions and inline surfaces both convert; a missing one would
    leave a segment of wall frictionless, and invisibly so.
    """
    name, _, _ = stand
    root = ET.parse(os.path.join(MODELS, name, 'model.sdf')).getroot()
    cols = root.findall('model/link/collision')
    assert len(cols) == SEGMENTS * 2
    for col in cols:
        assert col.find('surface/friction/ode/mu') is not None


# -- how the world uses them ------------------------------------------------

def _world_poses():
    root = ET.parse(WORLD).getroot()
    return {inc.find('name').text:
            [float(v) for v in inc.find('pose').text.split()]
            for inc in root.iter('include')}


@pytest.mark.parametrize('stand_name', sorted(STATIONS))
def test_stand_is_placed_on_its_bottle_station(stand_name):
    bottle_name = STATIONS[stand_name]
    poses = _world_poses()
    assert stand_name in poses, f'{stand_name} is not in the world'
    assert poses[stand_name][:3] == pytest.approx(poses[bottle_name][:3])
    assert poses[stand_name][3:] == pytest.approx([0.0, 0.0, 0.0])


@pytest.mark.parametrize('stand_name', sorted(STATIONS))
def test_stand_is_not_rotated_into_the_counter(stand_name):
    """The stand must sit at counter level, upright.

    Posed at the bottle's z, which is counter level, and with no floor under
    it, so the wall stands ON the counter rather than through it.
    """
    poses = _world_poses()
    assert poses[stand_name][2] == pytest.approx(0.9)


# -- what the action servers publish to the planning scene ------------------

# Which server puts each stand into the planning scene, and how it spells the
# number. Not all three belong to the pour: the beer is handled by
# bartender_open, whose sequence is the only one that touches it.
CONSUMERS = {
    'whiskey_stand': (('bartender_pour', 'bartender_pour',
                       'pour_action_server.py'), 'stand_radius={:.4f}'),
    'cola_stand': (('bartender_pour', 'bartender_pour',
                    'pour_action_server.py'), 'stand_radius={:.4f}'),
    'beer_stand': (('bartender_open', 'bartender_open', 'layout.py'),
                   'BEER_STAND_RADIUS = {:.4f}'),
}


@pytest.mark.parametrize('stand_name', sorted(STANDS))
def test_planning_scene_radius_matches_the_generator(stand_name):
    """Whoever publishes a stand hardcodes its radius; this checks the copy.

    Neither server can import this generator -- it is a script in a third
    package and is not on the path at runtime -- so the number is copied, and
    a copy needs a test. Getting it wrong is quiet: the planner simply
    believes the station is a different size from the thing standing there.
    """
    where, spelling = CONSUMERS[stand_name]
    source = open(os.path.join(REPO, 'ros2_ws', 'src', *where)).read()
    radius = outer_radius(stand_name)
    assert spelling.format(radius) in source, (
        f'{stand_name} outer radius {radius:.4f} is not in '
        f'{os.path.join(*where)}')


def test_the_stand_height_matches_the_generator():
    source = open(os.path.join(
        REPO, 'ros2_ws', 'src', 'bartender_pour', 'bartender_pour',
        'pour_action_server.py')).read()
    assert f'STAND_HEIGHT = {total_height():.3f}' in source
    layout = open(os.path.join(
        REPO, 'ros2_ws', 'src', 'bartender_open', 'bartender_open',
        'layout.py')).read()
    assert f'STAND_HEIGHT = {total_height():.3f}' in layout


def test_outer_radius_is_the_widest_part():
    """The planning scene gets one cylinder, so it has to be the outer one."""
    for name, spec in STANDS.items():
        inner_r = spec['bottle_radius'] + spec['clearance']
        widest = max(math.hypot(x, y) + WALL / 2.0
                     for r, _, _ in courses(inner_r)
                     for x, y, _, _ in segment_poses(r))
        assert outer_radius(name) >= widest - 1e-12
        assert outer_radius(name) == pytest.approx(inner_r + LEAD + WALL)


def test_lead_in_geometry_is_the_source_of_the_height():
    assert total_height() == pytest.approx(HOLD_H + LEAD_H)
