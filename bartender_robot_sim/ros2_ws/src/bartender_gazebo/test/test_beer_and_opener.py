"""Tests for the beer, its cap, the opener and the holster.

These four models are one design and the tests are mostly about that: the cap
is sized to the bottle's crown finish, the bell to the cap, the post to the
bell. Any one of them can be changed on its own without anything complaining,
and the failure that follows is mechanical and mute -- a bell that lands on
the cap instead of over it, a cap that starts the world 4mm inside the bottle
it is joined to.

The other thing protected here is the beer's collision stack, which is not
decoration. The bottle is held by its NECK, on the step under the straight
band, because two flat pads cannot hold its 64.4mm body: gripping the body
threw it 3.8 metres in the first run that tried. If the band stops being
straight, or the shoulder under it stops being a step, the grasp silently
goes back to being that.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo',
                                'scripts'))

import make_beer_and_opener as gen                      # noqa: E402

MODELS = os.path.join(REPO, 'models')
WORLD = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'worlds',
                     'bar_world.sdf')


def model_root(name):
    return ET.parse(os.path.join(MODELS, name, 'model.sdf')).getroot()


def collisions(name):
    """(collision name, pose, geometry element) for every collision."""
    out = []
    for col in model_root(name).iter('collision'):
        pose = col.find('pose')
        xyz = ([float(v) for v in pose.text.split()[:3]]
               if pose is not None else [0.0, 0.0, 0.0])
        out.append((col.get('name'), xyz, col.find('geometry')))
    return out


# -- the models on disk are the ones the generator makes --------------------

@pytest.mark.parametrize('name', sorted(gen.MODELS))
def test_checked_in_model_matches_the_generator(name):
    """Regenerating must be a no-op.

    These files are generated, and the generator is where the reasoning for
    every number in them lives. An edit made directly to the SDF survives
    until the next person runs the script, and then vanishes along with
    whatever it was for.
    """
    builder, _ = gen.MODELS[name]
    with open(os.path.join(MODELS, name, 'model.sdf')) as fh:
        assert fh.read() == builder()


@pytest.mark.parametrize('name', sorted(gen.MODELS))
def test_model_is_well_formed_xml(name):
    """Check with a conforming parser, which sdformat's is not.

    TinyXML2 accepts `--` inside a comment; the standard does not, and
    anything that parses these files with a conforming parser -- including
    every test here -- falls over on a file the simulator loads happily.
    """
    ET.parse(os.path.join(MODELS, name, 'model.sdf'))


@pytest.mark.parametrize('name', sorted(gen.MODELS))
def test_every_collision_carries_friction(name):
    """Friction is set inline, so a collision without it is frictionless.

    There is no default worth having here: a frictionless opener slides out
    of the pads, and a frictionless bottle cannot be held at all.
    """
    for col in model_root(name).iter('collision'):
        assert col.find('surface/friction/ode/mu') is not None, col.get('name')


@pytest.mark.parametrize('name,static', [('opener_holster', True),
                                         ('beer_bottle', False),
                                         ('beer_cap', False),
                                         ('bottle_opener', False)])
def test_static_where_it_should_be(name, static):
    element = model_root(name).find('model/static')
    assert (element is not None and element.text == 'true') == static


# -- the beer ---------------------------------------------------------------

def test_the_collision_bands_cover_the_whole_bottle():
    """No gaps, no overlaps: a gap is a place the pads pass through."""
    bands = gen.beer_bands()
    assert bands[0][0] == 0.0
    assert bands[-1][1] == pytest.approx(gen.BEER_HEIGHT, abs=1e-9)
    for (_, top, _), (bottom, _, _) in zip(bands, bands[1:]):
        assert top == pytest.approx(bottom, abs=1e-9)


def test_band_radii_are_measured_from_the_mesh():
    """Each band's radius is its widest vertex, not a number someone typed.

    Widest rather than mean, because every step in this profile is meant to
    be a stop, and a stop that is slightly proud still stops things.
    """
    profile = gen.beer_profile()
    for z0, z1, radius in gen.beer_bands():
        inside = [r for z, r in profile if z0 - 1e-9 <= z <= z1 + 1e-9]
        assert radius == pytest.approx(max(inside), abs=1e-12)


def test_the_grip_band_is_straight():
    """The pads close on it, so it must not be a cone.

    A cone between two flat pads is a wedge: it decides its own height in the
    grip and slides until it finds it.
    """
    z0, z1 = gen.BEER_GRIP_BAND
    inside = [r for z, r in gen.beer_profile() if z0 - 1e-9 <= z <= z1 + 1e-9]
    assert max(inside) - min(inside) < 0.002


def test_there_is_a_step_under_the_grip_band():
    """The feature the whole grasp depends on.

    Going down from the neck the bottle gets wider, so pads closed on the
    neck cannot pass the shoulder. That is what carries the bottle, and it is
    also the right way round for being pushed down on: the push drives the
    wedge further into the pads.
    """
    bands = {(z0, z1): r for z0, z1, r in gen.beer_bands()}
    grip = bands[gen.BEER_GRIP_BAND]
    below = max(r for (z0, z1), r in bands.items()
                if z1 <= gen.BEER_GRIP_BAND[0])
    assert below - grip > 0.005


def test_the_neck_narrows_above_the_grip():
    bands = {(z0, z1): r for z0, z1, r in gen.beer_bands()}
    grip = bands[gen.BEER_GRIP_BAND]
    above = max(r for (z0, z1), r in bands.items()
                if z0 >= gen.BEER_GRIP_BAND[1])
    assert above < grip


def test_the_grip_band_fits_between_the_open_pads():
    """85.00mm is the whole opening; the neck has to be comfortably inside it."""
    assert 2.0 * gen.beer_grip_radius() < 0.085 - 0.020


def test_the_bottle_declares_its_detachable_joint():
    """The cap can only come off if this plugin is here and names it right.

    child_model is matched against a world-level model NAME, so the world's
    <name> for the cap and this string are a pair held together by nothing
    but being spelled the same.
    """
    plugin = model_root('beer_bottle').find('model/plugin')
    assert plugin is not None
    assert 'DetachableJoint' in plugin.get('name')
    assert plugin.find('child_model').text == 'beer_cap'
    assert plugin.find('child_link').text == 'cap_link'
    assert plugin.find('detach_topic').text == gen.DETACH_TOPIC


def test_the_world_includes_the_cap_under_the_name_the_joint_uses():
    names = {inc.find('name').text for inc in ET.parse(WORLD).getroot()
             .iter('include')}
    assert 'beer_cap' in names
    assert 'beer_bottle' in names


def test_the_detach_topic_is_bridged():
    """The topic is ROS-side; without the bridge nothing reaches the plugin."""
    launch = os.path.join(REPO, 'ros2_ws', 'src', 'bartender_gazebo', 'launch',
                          'sim.launch.py')
    assert gen.DETACH_TOPIC in open(launch).read()


# -- the cap ----------------------------------------------------------------

def test_the_cap_sits_entirely_above_the_bottle():
    """Zero overlap with the bottle it is joined to.

    While the DetachableJoint holds them they are one skeleton and overlap is
    tolerated. The instant it lets go they are two bodies sharing space, and
    the solver's answer to that is to throw one of them. That is why the cap
    is a disc on the lip rather than a skirt over the bead, which is what a
    real one is.
    """
    caps = collisions('beer_cap')
    assert len(caps) == 1
    _, xyz, geometry = caps[0]
    length = float(geometry.find('cylinder/length').text)
    assert xyz[2] - length / 2.0 == pytest.approx(0.0, abs=1e-9)


def test_the_cap_is_wider_than_the_bottles_mouth():
    """It has to sit ON the lip, not drop through it."""
    finish = gen.beer_bands()[-1][2]
    assert gen.CAP_RADIUS > finish


def test_the_cap_is_light():
    """It shares a contact with the bell and the lip once it is free.

    A heavy one pushes them around; a real crown cap is about two grams.
    """
    assert gen.CAP_MASS < 0.005


# -- the opener -------------------------------------------------------------

def test_the_bell_is_boxes_and_not_a_mesh():
    """A ring's convex hull is a solid disc.

    Built as one mesh this would not fit over the cap at all -- it would sit
    on top of it -- because DART collides a dynamic body's mesh as its hull.
    The grooved fingertip pad taught this project that once already; see
    PAD_MODE 'groove' in render_bartender_urdf.py.
    """
    shapes = [geometry for name, _, geometry in collisions('bottle_opener')
              if name.startswith('bell_')]
    assert shapes, 'no bell collisions at all'
    for geometry in shapes:
        assert geometry.find('box') is not None
        assert geometry.find('mesh') is None


def test_the_bell_wall_has_no_gaps():
    """Sweep the bore and check a wall box is always outside it.

    Sixteen boxes laid on a circle overlap at the corners if their length
    comes from the outer tangent circle and leave sixteen slots if it does
    not. A slot is a place the cap escapes sideways from.
    """
    inner = gen.bell_inner_radius()
    boxes = [(xyz, yaw, size)
             for (xyz, yaw, size) in gen.bell_segment_boxes()
             if abs(xyz[2] - (gen.BELL_LEAD_H + gen.BELL_BORE_H / 2.0)) < 1e-9]
    assert len(boxes) == gen.BELL_SEGMENTS
    probe = inner + 0.0005
    for step in range(720):
        angle = 2.0 * math.pi * step / 720
        point = (probe * math.cos(angle), probe * math.sin(angle))
        assert any(_inside(point, xyz, yaw, size) for xyz, yaw, size in boxes), (
            f'gap in the bell wall at {math.degrees(angle):.1f} degrees')


def _inside(point, xyz, yaw, size):
    dx, dy = point[0] - xyz[0], point[1] - xyz[1]
    c, s = math.cos(-yaw), math.sin(-yaw)
    local = (c * dx - s * dy, s * dx + c * dy)
    return (abs(local[0]) <= size[0] / 2.0 + 1e-9
            and abs(local[1]) <= size[1] / 2.0 + 1e-9)


def test_the_bore_clears_the_cap():
    assert gen.bell_inner_radius() > gen.CAP_RADIUS
    assert gen.BELL_CLEARANCE == pytest.approx(
        gen.bell_inner_radius() - gen.CAP_RADIUS, abs=1e-12)


def test_the_mouth_is_wider_than_the_bore():
    """The chamfer is what funnels a slightly misplaced bell onto the cap.

    Without it the opener has to be brought down within BELL_CLEARANCE of the
    cap's centre, which is 2.5mm, against an arm that lands within about 3mm.
    Measured before it existed: 5.4mm off, bell resting on the cap, seating
    -1.6mm of a possible 16.
    """
    assert gen.BELL_LEAD > 0.0
    assert gen.bell_capture() > gen.BELL_CLEARANCE


def test_the_mouth_is_the_lower_course():
    """The cap comes in from below, so the wide end must be at the rim."""
    courses = gen.bell_courses()
    assert courses[0][1] == 0.0
    assert courses[0][0] > courses[1][0]


def test_the_crown_plate_closes_the_top_of_the_bell():
    """It is the face that transmits the push; a gap round it is a leak."""
    plate = [geometry for name, _, geometry in collisions('bottle_opener')
             if name == 'crown_plate_collision']
    assert len(plate) == 1
    radius = float(plate[0].find('cylinder/radius').text)
    assert radius >= gen.bell_outer_radius() - 1e-9


def test_the_shaft_is_where_the_pads_can_reach_it():
    """The grip point must be on the shaft, clear of the plate below it."""
    assert gen.shaft_grip_z() > gen.bell_height() + gen.PLATE_THICKNESS
    assert gen.shaft_grip_z() < gen.opener_height()


def test_the_shaft_fits_the_gripper_with_room():
    assert gen.SHAFT < 0.085 - 0.030


def test_the_shaft_is_square():
    """Flat faces stall the knuckle within a fraction of a millimetre.

    bartender_pour measured 0.2mm on the whiskey's flats against ~6mm for a
    round section, and this is the one object in the scene that is pushed
    against while gripped.
    """
    box = [geometry for name, _, geometry in collisions('bottle_opener')
           if name == 'shaft_collision'][0]
    size = [float(v) for v in box.find('box/size').text.split()]
    assert size[0] == pytest.approx(size[1], abs=1e-12)


# -- the holster ------------------------------------------------------------

def test_the_post_fits_inside_the_bell():
    assert gen.CAP_RADIUS < gen.bell_inner_radius()


def test_the_post_is_shorter_than_the_bell_is_deep():
    """So the opener rests on its rim and not on the post.

    Making them equal stands it on two things at once and which one wins is
    down to rounding. The rim on the counter is a 47mm annulus; the crown
    plate on a 32mm post is a balancing act.
    """
    assert gen.post_top() < gen.bell_height()


def test_the_post_has_a_lead_in():
    assert gen.POST_LEAD > 0.0
    assert gen.POST_LEAD_HEIGHT > 0.0


def test_seated_rim_is_one_bell_below_the_cap_top():
    assert gen.seated_rim_z(1.0) == pytest.approx(1.0 - gen.bell_height(),
                                                  abs=1e-12)
