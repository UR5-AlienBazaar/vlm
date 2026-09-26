"""Tests for the rendered robot description.

The description is not used as xacro writes it. Both launch files run it
through scripts/render_bartender_urdf.py, which widens the fingertip pads and
sets their friction, and the planner and the physics engine both read that
output -- so what this renders IS the robot, and anything wrong with it is
wrong everywhere at once.

Two things are being protected.

TWO ARMS, TWO GRIPPERS. Arm A keeps ur_description's bare names and arm B is
under `b_`; the reasoning for that asymmetry is in the xacro. The render
script used to match fingertips by exact name and insist on finding exactly
two. With a second gripper it has to match by suffix and find all four, and
the failure if it does not is silent in the worst way: one arm holding
bottles on stock pads at stock friction while the other has the tuned ones.

THE PADS THEMSELVES. The fingertip collisions and their friction are the
single most-tuned thing in this project -- PAD_MU alone is the difference
between 3/6 drinks poured and 7/7. Most of that tuning lives in comments;
these tests hold the parts of it that are structural.
"""
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
SCRIPTS = os.path.join(PACKAGE, 'scripts')
XACRO = os.path.join(PACKAGE, 'urdf', 'bartender.urdf.xacro')

sys.path.insert(0, SCRIPTS)

import render_bartender_urdf as render                  # noqa: E402


def rendered(mode='wide'):
    environment = dict(os.environ, BARTENDER_PAD_MODE=mode)
    result = subprocess.run(
        [sys.executable, os.path.join(SCRIPTS, 'render_bartender_urdf.py'),
         XACRO, 'sim_ignition:=true'],
        capture_output=True, text=True, env=environment)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


@pytest.fixture(scope='module')
def robot():
    return rendered()


def links(robot):
    return {link.get('name'): link for link in robot.findall('link')}


def fingertips(robot):
    return [name for name in links(robot) if render.is_fingertip(name)]


# -- both arms are there ----------------------------------------------------

def test_the_description_has_two_arms(robot):
    names = links(robot)
    for suffix in ('base_link', 'tool0', 'wrist_3_link', 'forearm_link'):
        assert suffix in names, f'arm A is missing {suffix}'
        assert 'b_' + suffix in names, f'arm B is missing {suffix}'


def test_both_arms_hang_off_the_same_world_link(robot):
    """One spawned model, one controller_manager, one planning scene.

    Arm B is attached to `world` rather than to anything of arm A's, but
    `world` is positioned by the ros_gz spawn, so arm B's origin is stated
    relative to arm A's base whether that is obvious or not. See the note in
    the xacro.
    """
    parents = {joint.find('child').get('link'): joint.find('parent').get('link')
               for joint in robot.findall('joint')}
    assert parents['base_link'] == 'world'
    assert parents['b_base_link'] == 'world'


def test_arm_b_is_somewhere_else(robot):
    """A second arm on top of the first is a very quiet mistake."""
    origin = [joint.find('origin') for joint in robot.findall('joint')
              if joint.find('child').get('link') == 'b_base_link'][0]
    xyz = [float(v) for v in origin.get('xyz').split()]
    assert max(abs(v) for v in xyz) > 0.3


def test_there_are_four_ros2_control_blocks(robot):
    """Two arms and two grippers, each with a distinct name.

    gz_ros2_control reads every one of them; duplicate names collide.
    """
    names = [block.get('name') for block in robot.findall('ros2_control')]
    assert len(names) == 4
    assert len(set(names)) == 4


def test_the_gz_plugin_is_declared_exactly_once(robot):
    """However many arms there are. It starts the one controller_manager."""
    plugins = [plugin for gazebo in robot.findall('gazebo')
               for plugin in gazebo.findall('plugin')
               if 'gz_ros2_control' in (plugin.get('filename') or '')]
    assert len(plugins) == 1


def test_the_two_arms_joints_do_not_collide(robot):
    """Every joint name unique, or the controllers fight over them."""
    names = [joint.get('name') for joint in robot.findall('joint')]
    assert len(names) == len(set(names))


@pytest.mark.parametrize(('name', 'reference', 'topic'), [
    ('arm_a_wrist_camera', 'tool0',
     '/bartender/arm_a/wrist_camera/image_raw'),
    ('arm_b_wrist_camera', 'b_tool0',
     '/bartender/arm_b/wrist_camera/image_raw'),
])
def test_wrist_camera_looks_along_gripper_approach(robot, name, reference,
                                                   topic):
    gazebo = next(tag for tag in robot.findall('gazebo')
                  if tag.find(f"sensor[@name='{name}']") is not None)
    sensor = gazebo.find(f"sensor[@name='{name}']")

    assert gazebo.get('reference') == reference
    assert sensor.findtext('topic') == topic
    assert sensor.findtext('camera/image/format') == 'R8G8B8'

    _roll, pitch, yaw = [float(value) for value in
                         sensor.findtext('pose').split()[3:]]
    # The first column of Rz(yaw) * Ry(pitch) * Rx(roll) transforms the
    # camera's local +X viewing axis into its parent tool0 frame.
    optical_axis = (
        math.cos(yaw) * math.cos(pitch),
        math.sin(yaw) * math.cos(pitch),
        -math.sin(pitch),
    )
    assert optical_axis == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)


# -- the pads ---------------------------------------------------------------

def test_every_fingertip_is_found(robot):
    """Four: two grippers' worth.

    Matched by suffix, because arm B's are prefixed. The script's own guard
    is that the count is a whole number of grippers -- an odd one means the
    suffix match caught half a gripper.
    """
    assert len(fingertips(robot)) == 4
    assert len(fingertips(robot)) % len(render.TIPS) == 0


def test_is_fingertip_is_not_fooled_by_the_prefix():
    for tip in render.TIPS:
        assert render.is_fingertip(tip)
        assert render.is_fingertip('b_' + tip)
    assert not render.is_fingertip('robotiq_85_left_finger_link')
    assert not render.is_fingertip('b_tool0')


def test_every_fingertip_gets_the_pad_friction(robot):
    """Including arm B's.

    <gazebo reference> is matched by LINK NAME, so a prefixed link needs its
    own tag; there is no inheritance. A missing one converts to a collision
    with no <mu> at all, silently -- which is how the whole 'friction tags do
    nothing' misreading started.
    """
    tagged = {gazebo.get('reference') for gazebo in robot.findall('gazebo')
              if gazebo.find('mu1') is not None}
    assert set(fingertips(robot)) <= tagged
    for gazebo in robot.findall('gazebo'):
        if gazebo.get('reference') in fingertips(robot):
            assert gazebo.find('mu1').text == str(render.PAD_MU)
            assert gazebo.find('mu2').text == str(render.PAD_MU)


def test_the_friction_is_a_coefficient_and_not_the_stock_idiom(robot):
    """The stock description asks for 100000, which DART takes literally.

    It is a Gazebo-Classic way of writing "never slip" and it is not a
    coefficient. Rubber on glass is a small number.
    """
    assert 0.5 < render.PAD_MU < 5.0


def test_the_wings_are_added_and_the_stock_mesh_is_kept(robot):
    """'wide' is ADDITIVE, which is the whole reason it is the default.

    The contact the system is tuned against is the stock mesh's. The wings
    only extend it along the bottle's axis, so the change is confined to the
    part that is new; replacing the mesh with primitives is what 'flat' does
    and it is a much bigger change than "the pads are wider".
    """
    for name in fingertips(robot):
        link = links(robot)[name]
        shapes = link.findall('collision/geometry')
        assert any(shape.find('mesh') is not None for shape in shapes), name
        boxes = [shape for shape in shapes if shape.find('box') is not None]
        assert len(boxes) == 2, f'{name} should have a wing above and below'


def test_the_wings_bring_the_pad_to_the_intended_width(robot):
    """Two wings plus the stock 22mm face, to PAD_WIDTH along the bottle.

    Both stock faces are 22mm, so this holds on either side even though the
    two wings are NOT the same size as each other -- they are sized to put
    both pads on a common band, which is a different test below.

    Widened in y and only in y: x is the closing axis and eating into it
    costs approach clearance the whiskey bottle does not have (3.9mm a side),
    and z is already full in both directions.
    """
    for name in fingertips(robot):
        link = links(robot)[name]
        widths = [float(shape.find('box').get('size').split()[1])
                  for shape in link.findall('collision/geometry')
                  if shape.find('box') is not None]
        assert len(widths) == 2
        assert sum(widths) + 0.022 == pytest.approx(render.PAD_WIDTH, abs=5e-4)


def test_the_wings_are_flat_against_the_pad_plane(robot):
    """Not proud of it, or the gripper's 85.00mm opening is reduced.

    Both wings on a fingertip sit at the same x as each other and differ only
    in y -- one above the pad, one below.
    """
    for name in fingertips(robot):
        link = links(robot)[name]
        placed = [[float(v) for v in shape.find('origin').get('xyz').split()]
                  for shape in link.findall('collision')
                  if shape.find('geometry/box') is not None]
        assert len(placed) == 2
        assert placed[0][0] == pytest.approx(placed[1][0], abs=1e-9)
        assert placed[0][2] == pytest.approx(placed[1][2], abs=1e-9)
        assert placed[0][1] * placed[1][1] < 0, 'wings should straddle the pad'


def test_the_widened_pad_still_fits_every_bottle_it_has_to_clear(robot):
    """The width is the derived limit, not a number that outlived its reason.

    This used to assert a literal 0.034 with "the cola's waist is the binding
    constraint" for a reason, and that stopped being true when the beer
    arrived -- its shoulder is tighter than the cola's rim. Asserting the
    derivation instead means the next bottle either fits or fails here.
    """
    assert render.PAD_HALF_WIDTH == pytest.approx(
        render.pad_half_width_limit(), abs=1e-9)
    assert render.PAD_WIDTH == pytest.approx(2 * render.PAD_HALF_WIDTH)


def test_every_blocking_band_edge_keeps_its_landing_margin(robot):
    """Spelled out per bottle, because min() hides which one is binding.

    A blocking edge is one the bottle steps OUTWARD at. Reach past it and the
    pad lands on the step and never touches the band -- the cola grips its
    base or bulge instead of its waist, the beer its shoulder instead of its
    neck.
    """
    # The epsilon is not slack. The cola's waist binds EXACTLY -- its margin
    # comes out at 3.5000mm at both ends, because PAD_HALF_WIDTH is that edge
    # and nothing else -- so an exact >= fails on float representation alone.
    slack = 1e-9
    for name, low, high, grasp, low_blocks, high_blocks in render.GRIP_BANDS:
        if low_blocks:
            assert grasp - render.PAD_HALF_WIDTH >= low + \
                render.PAD_LANDING_MARGIN - slack, f'{name} below'
        if high_blocks:
            assert grasp + render.PAD_HALF_WIDTH <= high - \
                render.PAD_LANDING_MARGIN + slack, f'{name} above'


def test_the_landing_margin_beats_how_accurately_the_arm_lands(robot):
    """3mm, measured on the teach pendant: a 50mm jog moved 47mm.

    The margin is against the ARM, not against the bottle. A pad edge closer
    to a blocking step than the arm's own error will sometimes be past it,
    and that failure looks like bad luck with the physics rather than like a
    dimension being wrong.
    """
    assert render.PAD_LANDING_MARGIN >= 0.003


def test_both_fingertips_land_on_the_same_band(robot):
    """The stock pads are 0.85mm out of register; the wings cancel that.

    Only the overlap of the two pads is contact, so a mismatch costs width at
    one end and puts a couple on whatever is held. This is the invariant that
    makes the wings different sizes on the two sides -- if a change makes them
    symmetric again, it has reintroduced the mismatch.
    """
    bands = {}
    for name in fingertips(robot):
        link = links(robot)[name]
        edges = []
        for col in link.findall('collision'):
            shape = col.find('geometry/box')
            if shape is None:
                continue
            width = float(shape.get('size').split()[1])
            centre = float(col.find('origin').get('xyz').split()[1])
            edges += [centre - width / 2, centre + width / 2]
        bands[name] = (min(edges), max(edges))
    for name, (low, high) in bands.items():
        assert low == pytest.approx(-render.PAD_HALF_WIDTH, abs=1e-9), name
        assert high == pytest.approx(render.PAD_HALF_WIDTH, abs=1e-9), name


def test_the_wing_boxes_carry_no_name_attribute(robot):
    """Naming them makes the friction tag match fail, silently.

    sdformat matches <gazebo reference> surface properties against collisions
    BY NAME. Measured on this exact model: named collisions convert with zero
    <mu> elements, the identical geometry unnamed converts with all eight.
    """
    for name in fingertips(robot):
        for collision in links(robot)[name].findall('collision'):
            if collision.find('geometry/box') is not None:
                assert collision.get('name') is None


# -- the control mode -------------------------------------------------------

def test_mesh_mode_leaves_the_pads_alone_but_still_sets_friction():
    """The control for 'wide'. It must differ in exactly one thing."""
    robot = rendered('mesh')
    for name in fingertips(robot):
        link = links(robot)[name]
        assert not [shape for shape in link.findall('collision/geometry')
                    if shape.find('box') is not None]
    tagged = {gazebo.get('reference') for gazebo in robot.findall('gazebo')
              if gazebo.find('mu1') is not None}
    assert set(fingertips(robot)) <= tagged


def test_an_unknown_pad_mode_is_refused():
    """Rather than quietly shipping stock pads."""
    result = subprocess.run(
        [sys.executable, os.path.join(SCRIPTS, 'render_bartender_urdf.py'),
         XACRO, 'sim_ignition:=true'],
        capture_output=True, text=True,
        env=dict(os.environ, BARTENDER_PAD_MODE='nonsense'))
    assert result.returncode != 0
