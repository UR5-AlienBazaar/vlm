#!/usr/bin/env python3
"""Capture randomized workcell scenes from the Gazebo twin for the VLM dataset.

Run inside the sim container while the twin is up (scripts/workcell_vlm_twin.sh):
    python3 scripts/capture_workcell_scenes.py data/workcell_raw/<gen> --scenes 50 --seed 3 \\
        [--params params.json]

Each scene: the bottles (in workcell_world.sdf) and some distractors are
placed on the table or knocked over, the lighting and colours change, the
mock arm moves to a pose the twin copies (sometimes hovering over a bottle so
it hides part of it, sometimes holding one), one camera is put at a random viewpoint, and one RGB +
segmentation pair is saved with the objects' settled poses. The layout is the
contract with vlm/training (vlm_labels.py and the harness):

    <out>/<scene>/scene.json    bottles, glasses ([]), camera, appearance, params
    <out>/<scene>/0000.json     in_gripper, settled object poses, arm
    <out>/<scene>/0000_cam_rgb.png, 0000_cam_labels.png   (labels: bottles 1-3, distractors 20-29)

--params takes a JSON file overriding SCENE_PARAMS, which is how the VLM
harness steers what gets generated towards the cases the model gets wrong.
Re-running the same command after a crash resumes: finished scenes are
skipped and their random draws replayed, so the result is the same as an
uninterrupted run. Every run appends one summary line (sim commit, params,
what it captured and how long it took) to capture_runs.jsonl next to <out>.
"""
import argparse
import collections
import colorsys
import json
import math
import random
import re
import subprocess
import time
from pathlib import Path

import numpy as np

WORLD = 'workcell_world'
TABLE_Z = 0.75  # workcell_twin.launch.py table_height; the table's corner is the Gazebo origin
TABLE = (1.40, 0.70)
ARM_BASE = (0.35, 0.35)
# In front of the arm, off its base and 8cm in from the edges.
PLACE_X = (0.55, 1.30)
PLACE_Y = (0.08, 0.62)
MIN_GAP = 0.11
HIDDEN_Z = -5.0
UPRIGHT_MAX_TILT_DEG = 20

# Bottle -> its model in workcell_world.sdf. The real workcell has no glass.
OBJECTS = {'whiskey': 'jack_daniels_bottle', 'cola': 'cola_bottle', 'beer': 'beer_bottle'}
BEER_CAP_HEIGHT = 0.2581  # the beer's lip above its base; see bar_world.sdf
# Distractor i gets label 20 + i: vlm_labels treats 20-29 as distractors, and
# separate labels keep two distractors from merging into one box. Each is a
# list of (geometry, height of its centre above the model's base), so the base
# is the origin and a distractor stands on the table like a bottle does.
# `wine` is a bottle that is none of ours: the hard negative.
DISTRACTORS = [
    ('box', [('<box><size>0.07 0.07 0.16</size></box>', 0.08)]),
    ('can', [('<cylinder><radius>0.033</radius><length>0.12</length></cylinder>', 0.06)]),
    ('carton', [('<box><size>0.10 0.05 0.20</size></box>', 0.10)]),
    ('wine', [('<cylinder><radius>0.037</radius><length>0.21</length></cylinder>', 0.105),
              ('<cylinder><radius>0.014</radius><length>0.09</length></cylinder>', 0.255)]),
    ('mug', [('<cylinder><radius>0.045</radius><length>0.10</length></cylinder>', 0.05)]),
    ('ball', [('<sphere><radius>0.04</radius></sphere>', 0.04)]),
    ('spray', [('<cylinder><radius>0.025</radius><length>0.22</length></cylinder>', 0.11)]),
    ('book', [('<box><size>0.20 0.14 0.035</size></box>', 0.0175)]),
]

SCENE_PARAMS = {
    'bottle_p': 0.7,          # each bottle is on the table
    'fallen_p': 0.08,         # a present bottle lies on its side
    'distractors': [0.3, 0.35, 0.25, 0.1],  # P(0, 1, 2, 3 distractors)
    'distractor_kinds': None,  # restrict to these DISTRACTORS names, e.g. ["wine"]
    'block_p': 0.4,           # a distractor stands between the arm and a bottle
    'arm_pose': {'home': 0.25, 'over_table': 0.4, 'over_bottle': 0.35},
    'camera': {'front': 0.35, 'side': 0.25, 'overhead': 0.2, 'corner': 0.2},
    'appearance_p': 0.8,      # lighting and colours are randomized; else the world's own look
    'held_p': 0.15,           # the arm holds one of the bottles over the table
}
# Viewpoint families: eye (centre, jitter) and the point it looks at. The real
# camera's mount is not decided, so every scene draws one; the family is
# recorded so eval can be split by it.
CAMERAS = {
    'front': ((1.80, 0.35, 1.25), (0.10, 0.15, 0.10), (0.80, 0.35, TABLE_Z)),
    'side': ((0.85, -0.50, 1.30), (0.15, 0.10, 0.10), (0.85, 0.35, TABLE_Z)),
    'overhead': ((0.85, 0.35, 1.95), (0.10, 0.08, 0.10), (0.85, 0.35, TABLE_Z)),
    'corner': ((1.70, -0.35, 1.40), (0.10, 0.10, 0.10), (0.80, 0.35, TABLE_Z)),
}
CAM_SIZE = (640, 400)
CAM_HFOV = 1.2566  # OAK-D, the same as the bar's stand camera
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
# The arm reaches along world angle pan - pi/2 (measured on the twin: pan 0
# reaches -y, pan pi/2 reaches +x, down the table), because arm_yaw is -pi/2.
PAN_OFFSET = math.pi / 2
# UR5e: shoulder height, upper arm, forearm, and wrist_3 below wrist_1 when the
# tool points down. REACH_FUDGE is the horizontal distance the wrist offsets
# add, measured: planar 0.55 put wrist_3 at 0.66 from the base.
SHOULDER_Z, UPPER_ARM, FOREARM, WRIST_DROP = 0.1625, 0.425, 0.3922, 0.0997
REACH_FUDGE = 0.11
# d4: wrist_3 sits this far to the side of the arm's plane, so aiming the pan
# straight at a bottle put the wrist 0.29 rad off it at 0.46 reach (measured).
WRIST_LATERAL = 0.1333
# Wrist this high over the table keeps the gripper's fingers just above a
# bottle's top (~0.27), so a hover hides the bottle rather than hits it.
HOVER_Z = (0.45, 0.55)
# A held bottle is a static, collision-free copy of it (the mock gripper
# cannot grasp) hung under the wrist: wrist_3 to fingertip centre is
# ~0.10 (flange) + ~0.14 (2F-85), gripped ~0.12 up the bottle. Estimated
# from the datasheets; check by eye when changing it.
HOLD_DROP = 0.36
GRIP_OPEN, GRIP_CLOSED = 0.02, 0.12  # knuckle rad; 0.12 closes on a ~77mm bottle
MODELS_DIR = Path(__file__).resolve().parents[1] / 'models'
TABLE_COLOURS = [(0.62, 0.50, 0.36), (0.45, 0.33, 0.22), (0.80, 0.80, 0.78),
                 (0.30, 0.30, 0.32), (0.12, 0.12, 0.12), (0.55, 0.60, 0.65)]
TMP = Path('/tmp/workcell_vlm')


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def look_at(eye, target):
    """roll, pitch, yaw that point a Gazebo camera (it looks along +x) from eye at target."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    return 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def tilt_deg(q):
    up_z = 1 - 2 * (q['x'] ** 2 + q['y'] ** 2)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def _choice(rng, weights):
    return rng.choices(list(weights), list(weights.values()))[0]


def sample_layout(rng, params):
    """Where each object goes this scene. Pure, so it is testable without a sim."""
    placed = {}

    def free_spot():
        for _ in range(200):
            p = (rng.uniform(*PLACE_X), rng.uniform(*PLACE_Y))
            if all(math.dist(p, q['xy']) >= MIN_GAP for q in placed.values()):
                return p
        return None

    for name in OBJECTS:
        if rng.random() < params['bottle_p']:
            spot = free_spot()
            if spot:
                placed[name] = {'xy': spot, 'yaw': rng.uniform(-math.pi, math.pi),
                                'fallen': rng.random() < params['fallen_p']}
    bottles = [b for b in OBJECTS if b in placed]
    kinds = [i for i, (k, _) in enumerate(DISTRACTORS)
             if not params.get('distractor_kinds') or k in params['distractor_kinds']]
    n = rng.choices(range(len(params['distractors'])), params['distractors'])[0]
    for i in rng.sample(kinds, min(n, len(kinds))):
        spot = None
        if not any(k.startswith('distractor') for k in placed) and bottles and rng.random() < params['block_p']:
            bx, by = placed[rng.choice(bottles)]['xy']
            ax, ay = ARM_BASE
            d = math.dist((ax, ay), (bx, by))
            back = rng.uniform(0.09, 0.14) / d  # just in front of it, on the arm's side
            spot = (bx + (ax - bx) * back + rng.uniform(-0.02, 0.02),
                    by + (ay - by) * back + rng.uniform(-0.02, 0.02))
        placed[f'distractor{i}'] = {'xy': spot or free_spot() or (1.2, 0.1),
                                    'yaw': rng.uniform(-math.pi, math.pi), 'fallen': False,
                                    'kind': DISTRACTORS[i][0]}
    return placed


def sample_held(rng, params, layout):
    """Maybe take one placed, standing bottle off the table and into the gripper."""
    standing = [b for b in OBJECTS if b in layout and not layout[b]['fallen']]
    if not standing or rng.random() >= params['held_p']:
        return None
    held = rng.choice(standing)
    del layout[held]
    return held


def arm_ik(reach, height):
    """Shoulder lift and elbow that put wrist_3 `reach` from the base and `height`
    above the table with the tool pointing down, or None if out of reach."""
    r = reach - REACH_FUDGE
    dz = height + WRIST_DROP - SHOULDER_Z
    c = (r * r + dz * dz - UPPER_ARM ** 2 - FOREARM ** 2) / (2 * UPPER_ARM * FOREARM)
    if not -1 <= c <= 1:
        return None
    elbow = math.acos(c)
    lift = -(math.atan2(dz, r) + math.atan2(FOREARM * math.sin(elbow), UPPER_ARM + FOREARM * math.cos(elbow)))
    return lift, elbow


def arm_joints(pan, lift, elbow, wrist_3=0.0):
    # wrist_1 keeps the tool pointing straight down whatever lift and elbow are.
    return [pan, lift, elbow, -math.pi / 2 - (lift + elbow), -math.pi / 2, wrist_3]


def sample_arm(rng, params, layout, held=None):
    if held:
        # High enough that the bottle hanging below the wrist clears the table.
        ik = arm_ik(rng.uniform(0.40, 0.70), rng.uniform(HOLD_DROP + 0.08, HOLD_DROP + 0.25))
        # +-0.6 rad keeps the wrist over the table; +-1.0 put it past the long edges.
        return 'holding', held, arm_joints(rng.uniform(-0.6, 0.6) + PAN_OFFSET, *ik, rng.uniform(-math.pi, math.pi))
    kind = _choice(rng, params['arm_pose'])
    bottles = [b for b in OBJECTS if b in layout]
    if kind == 'over_bottle' and bottles:
        target = rng.choice(bottles)
        bx, by = layout[target]['xy']
        dx, dy = bx - ARM_BASE[0], by - ARM_BASE[1]
        # Short of it or over it: either way the arm is between it and most cameras.
        reach = math.hypot(dx, dy) - rng.uniform(0.0, 0.10)
        ik = arm_ik(reach, rng.uniform(*HOVER_Z))
        if ik:
            pan = math.atan2(dy, dx) + PAN_OFFSET - math.asin(min(1.0, WRIST_LATERAL / reach))
            return kind, target, arm_joints(pan, *ik, rng.uniform(-math.pi, math.pi))
        kind = 'over_table'
    if kind == 'home' or kind == 'over_bottle':
        return 'home', None, HOME
    ik = None
    while ik is None:  # far and high together is out of reach; draw again
        ik = arm_ik(rng.uniform(0.35, 0.75), rng.uniform(0.35, 0.65))
    return kind, None, arm_joints(rng.uniform(-1.1, 1.1) + PAN_OFFSET, *ik, rng.uniform(-math.pi, math.pi))


def sample_camera(rng, params):
    family = _choice(rng, params['camera'])
    centre, jitter, target = CAMERAS[family]
    eye = tuple(c + rng.uniform(-j, j) for c, j in zip(centre, jitter))
    target = (target[0] + rng.uniform(-0.1, 0.1), target[1] + rng.uniform(-0.08, 0.08), target[2])
    return family, eye, look_at(eye, target)


def _rgb(rng, sat=(0.3, 0.9), val=(0.25, 0.95)):
    return tuple(round(c, 3) for c in colorsys.hsv_to_rgb(rng.random(), rng.uniform(*sat), rng.uniform(*val)))


def sample_appearance(rng, params):
    """Sun, table, floor and distractor colours; None keeps the world's own look."""
    distractors = {f'distractor{i}': _rgb(rng) for i in range(len(DISTRACTORS))}
    if rng.random() >= params['appearance_p']:
        return None, distractors
    warm = rng.uniform(-0.25, 0.25)  # <0 bluish daylight, >0 warm bar lighting
    elevation = rng.uniform(0.5, 1.4)
    azimuth = rng.uniform(-math.pi, math.pi)
    return {
        'sun': {'intensity': round(rng.uniform(0.5, 1.6), 3),
                'diffuse': tuple(round(min(1.0, 0.85 + d), 3) for d in (warm, 0.0, -warm)),
                'direction': tuple(round(v, 3) for v in (math.cos(azimuth) * math.cos(elevation),
                                                         math.sin(azimuth) * math.cos(elevation),
                                                         -math.sin(elevation)))},
        'table': tuple(round(min(1.0, max(0.0, c + rng.uniform(-0.06, 0.06))), 3)
                       for c in rng.choice(TABLE_COLOURS)),
        'ground': (round(rng.uniform(0.15, 0.7), 3),) * 3,
    }, distractors


DEFAULT_LOOK = {'sun': {'intensity': 1.0, 'diffuse': (0.8, 0.8, 0.8), 'direction': (-0.5, 0.1, -0.9)},
                'table': (0.62, 0.50, 0.36), 'ground': (0.4, 0.4, 0.4)}


def _ign(service, reqtype, req, reptype='ignition.msgs.Boolean'):
    return subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
                           '--reptype', reptype, '--timeout', '5000', '--req', req],
                          check=True, capture_output=True, text=True).stdout


def set_pose(model, x, y, z, rpy=(0.0, 0.0, 0.0)):
    qx, qy, qz, qw = quat(*rpy)
    _ign('set_pose', 'ignition.msgs.Pose',
         f'name: "{model}" position {{x: {x} y: {y} z: {z}}} '
         f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')


def _colour(rgb):
    r, g, b = rgb
    return f'{{r: {r} g: {g} b: {b} a: 1}}'


def set_colour(visual_id, rgb):
    dim = tuple(c * 0.6 for c in rgb)
    _ign('visual_config', 'ignition.msgs.Visual',
         f'id: {visual_id} material {{ambient {_colour(dim)} diffuse {_colour(rgb)}}}')


def set_sun(sun):
    x, y, z = sun['direction']
    _ign('light_config', 'ignition.msgs.Light',
         f'name: "sun" type: DIRECTIONAL diffuse {_colour(sun["diffuse"])} '
         f'specular {{r: 0.2 g: 0.2 b: 0.2 a: 1}} direction {{x: {x} y: {y} z: {z}}} '
         f'cast_shadows: true intensity: {sun["intensity"]}')


def visual_ids():
    """Visual name -> entity id, for the uniquely named visuals set_colour recolours."""
    scene = _ign('scene/info', 'ignition.msgs.Empty', '', reptype='ignition.msgs.Scene')
    return {name: int(i) for name, i in re.findall(r'name: "([^"]+)"\s*\n\s*id: (\d+)', scene)}


def spawn(name, sdf, z=HIDDEN_Z):
    path = TMP / f'{name}.sdf'
    path.write_text(sdf)
    _ign('create', 'ignition.msgs.EntityFactory',
         f'sdf_filename: "{path}" name: "{name}" allow_renaming: false '
         f'pose {{position {{x: 0 y: 0 z: {z}}}}}')


LABEL_PLUGIN = '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{}</label></plugin>'


def distractor_sdf(name, parts, label):
    shapes = ''.join(
        f'<collision name="{name}_c{k}"><pose>0 0 {z} 0 0 0</pose><geometry>{g}</geometry></collision>'
        # A starting material: visual_config recolours it, but turns a visual without one black.
        f'<visual name="{name}_v{k}"><pose>0 0 {z} 0 0 0</pose><geometry>{g}</geometry>'
        '<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>'
        for k, (g, z) in enumerate(parts))
    return (f'<sdf version="1.9"><model name="{name}"><link name="link">'
            '<inertial><mass>0.2</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz>'
            f'</inertia></inertial>{shapes}</link>{LABEL_PLUGIN.format(label)}</model></sdf>')


def held_sdf(bottle, label):
    """The bottle's own model, static and visual-only, under the name held_<bottle>."""
    model = OBJECTS[bottle]
    sdf = (MODELS_DIR / model / 'model.sdf').read_text()
    for tag in ('collision', 'inertial', 'plugin'):
        sdf = re.sub(rf'<{tag}\b.*?</{tag}>', '', sdf, flags=re.S)
    # Mesh URIs are relative to model.sdf, and the copy is written elsewhere.
    sdf = sdf.replace('<uri>meshes/', f'<uri>model://{model}/meshes/')
    sdf = re.sub(r'<model name="[^"]*">', f'<model name="held_{bottle}"><static>true</static>', sdf, count=1)
    head, _, tail = sdf.rpartition('</model>')
    return head + LABEL_PLUGIN.format(label) + '</model>' + tail


def camera_sdf():
    w, h = CAM_SIZE
    cam = (f'<camera><horizontal_fov>{CAM_HFOV}</horizontal_fov><image><width>{w}</width>'
           f'<height>{h}</height>{{fmt}}</image><clip><near>0.1</near><far>6</far></clip>{{seg}}</camera>')
    return ('<sdf version="1.9"><model name="vlm_camera"><static>true</static><link name="link">'
            '<sensor name="rgb" type="camera"><always_on>true</always_on><update_rate>10</update_rate>'
            '<topic>/vlm_cam/rgb</topic>'
            + cam.format(fmt='<format>R8G8B8</format>', seg='') + '</sensor>'
            '<sensor name="seg" type="segmentation"><always_on>true</always_on><update_rate>10</update_rate>'
            '<topic>/vlm_cam/seg</topic>'
            + cam.format(fmt='', seg='<segmentation_type>semantic</segmentation_type>') + '</sensor>'
            '</link></model></sdf>')


class Sim:
    def __init__(self):
        import rclpy
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        from rclpy.action import ActionClient
        from sensor_msgs.msg import Image, JointState
        from tf2_msgs.msg import TFMessage

        self.bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/vlm_cam/rgb@sensor_msgs/msg/Image[ignition.msgs.Image',
             '/vlm_cam/seg/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image',
             f'/world/{WORLD}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('capture_workcell_scenes')
        self.latest, self.poses, self.joints = {}, {}, {}
        self.node.create_subscription(Image, '/vlm_cam/rgb', lambda m: self.latest.__setitem__('rgb', m), 2)
        self.node.create_subscription(Image, '/vlm_cam/seg/labels_map',
                                      lambda m: self.latest.__setitem__('labels', m), 2)
        self.node.create_subscription(
            TFMessage, f'/world/{WORLD}/dynamic_pose/info',
            lambda m: self.poses.update({t.child_frame_id: t.transform for t in m.transforms}), 2)
        self.node.create_subscription(
            JointState, '/twin/joint_states',
            lambda m: self.joints.update(zip(m.name, m.position)), 2)
        self.arm = ActionClient(self.node, FollowJointTrajectory, '/ur_arm_controller/follow_joint_trajectory')
        self.FollowJointTrajectory = FollowJointTrajectory
        self.gripper = ActionClient(self.node, GripperCommand, '/gripper_controller/gripper_cmd')
        self.GripperCommand = GripperCommand

    def close(self):
        self.bridge.terminate()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def move_arm(self, joints, seconds=2.0, wait=4.0):
        """Move the mock robot; twin_mirror makes the Gazebo arm follow.

        True once the twin is there. The twin can fall short, e.g. pressed on
        a bottle or the table, and the pose that counts is the one it holds.
        """
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        goal = self.FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=list(joints), time_from_start=Duration(sec=int(seconds), nanosec=int(seconds % 1 * 1e9)))]
        if not self.arm.wait_for_server(timeout_sec=30):
            raise RuntimeError('ur_arm_controller action server not up')
        sent = self.arm.send_goal_async(goal)
        while not sent.done():
            self.spin(0.05)
        done = sent.result().get_result_async()
        end = time.monotonic() + seconds + 10
        while not done.done() and time.monotonic() < end:
            self.spin(0.05)
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if all(abs(self.joints.get(j, 1e9) - q) < 0.03 for j, q in zip(ARM_JOINTS, joints)):
                return True
            self.spin(0.1)
        return False

    def grip(self, position):
        """Open or close the mock gripper; the twin copies the knuckle. Fire and forget."""
        if self.gripper.wait_for_server(timeout_sec=5):
            goal = self.GripperCommand.Goal()
            goal.command.position, goal.command.max_effort = position, 50.0
            self.gripper.send_goal_async(goal)

    def twin_joints(self):
        return [round(self.joints.get(j, float('nan')), 3) for j in ARM_JOINTS]

    def frame(self, timeout=15.0):
        self.latest.clear()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            rgb, labels = self.latest.get('rgb'), self.latest.get('labels')
            if rgb and labels and rgb.header.stamp == labels.header.stamp:
                return rgb, labels
        raise TimeoutError('camera did not deliver a matching rgb + labels pair')

    def pose(self, name):
        t = self.poses.get(name)
        if t is None:
            return None
        r = t.rotation
        q = {'x': r.x, 'y': r.y, 'z': r.z, 'w': r.w}
        return {'xyz': [round(t.translation.x, 4), round(t.translation.y, 4), round(t.translation.z, 4)],
                'tilt_deg': round(tilt_deg(q), 1)}


def model_name(name):
    return OBJECTS.get(name, name)


def place(model, spot, hide_x):
    """Stand (or lay) a model at its layout spot; None hides it under the floor."""
    if spot is None:
        pose = ((hide_x, 10, HIDDEN_Z), (0.0, 0.0, 0.0))  # apart, so hidden objects do not collide
    elif spot['fallen']:
        pose = ((*spot['xy'], TABLE_Z + 0.045), (math.pi / 2, 0.0, spot['yaw']))
    else:
        pose = ((*spot['xy'], TABLE_Z + 0.002), (0.0, 0.0, spot['yaw']))
    set_pose(model, *pose[0], pose[1])
    if model == OBJECTS['beer']:
        # The cap rides on a joint; teleporting only the bottle would tear it off.
        set_pose('beer_cap', *cap_xyz(*pose), pose[1])


def cap_xyz(base, rpy):
    """Where the beer's cap sits for a bottle whose base is at `base`, turned by roll/yaw."""
    roll, _, yaw = rpy
    up = (math.sin(roll) * math.sin(yaw), -math.sin(roll) * math.cos(yaw), math.cos(roll))
    return tuple(c + BEER_CAP_HEIGHT * u for c, u in zip(base, up))


def settled_objects(sim, models):
    """Where each object actually ended up, after physics, in table coordinates."""
    out = {}
    for name, model in models.items():
        p = sim.pose(model)
        if p is None or p['xyz'][2] < TABLE_Z - 0.3:
            out[name] = {'on_table': False}
            continue
        x, y, _ = p['xyz']
        out[name] = {'on_table': 0 <= x <= TABLE[0] and 0 <= y <= TABLE[1],
                     'xy': [x, y], 'upright': p['tilt_deg'] < UPRIGHT_MAX_TILT_DEG,
                     'tilt_deg': p['tilt_deg']}
    return out


def arm_state(sim):
    """The twin's own joints and its wrist in table coordinates (link poses are
    reported relative to the robot model, whose origin is the table's corner)."""
    wrist = sim.pose('wrist_3_link')
    return {'joints': sim.twin_joints(), 'wrist_xyz': wrist and wrist['xyz']}


def apply_appearance(ids, look, distractor_colours):
    look = look or DEFAULT_LOOK
    set_sun(look['sun'])
    set_colour(ids['workcell_table_visual'], look['table'])
    set_colour(ids['ground_visual'], look['ground'])
    for name, rgb in distractor_colours.items():
        for visual, i in ids.items():
            if visual.startswith(f'{name}_v'):
                set_colour(i, rgb)


def replay(rng, params):
    """Draw what a scene draws, in the same order as main(), without the sim."""
    layout = sample_layout(rng, params)
    held = sample_held(rng, params, layout)
    sample_appearance(rng, params)
    sample_arm(rng, params, layout, held)
    sample_camera(rng, params)


def log_run(out, opts, params, started, scenes, resumed, error=None):
    """One line per capture run in <out>/../capture_runs.jsonl: the run log to look at
    before the scenes themselves."""
    commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                            cwd=Path(__file__).parent).stdout.strip() or None
    count = lambda key: dict(collections.Counter(s[key] for s in scenes))  # noqa: E731
    line = {
        'finished_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), 'out': str(out),
        'seed': opts.seed, 'requested': opts.scenes, 'captured': len(scenes), 'resumed': resumed,
        'error': error, 'sim_commit': commit, 'wall_s': round(time.monotonic() - started, 1),
        's_per_scene': round(sum(s['seconds'] for s in scenes) / len(scenes), 1) if scenes else None,
        'camera': count('camera'), 'arm': count('arm'),
        'arm_reached': sum(s['reached'] for s in scenes), 'held': sum(s['held'] is not None for s in scenes),
        'params': params,
    }
    with open(Path(out).parent / 'capture_runs.jsonl', 'a') as f:
        f.write(json.dumps(line) + '\n')


def save(scene_dir, rgb, labels, frame):
    from PIL import Image as PILImage
    img = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, -1)
    lab = np.frombuffer(labels.data, np.uint8).reshape(labels.height, labels.width, -1)[:, :, 0]
    PILImage.fromarray(img[:, :, :3]).save(scene_dir / '0000_cam_rgb.png')
    PILImage.fromarray(lab).save(scene_dir / '0000_cam_labels.png')
    (scene_dir / '0000.json').write_text(json.dumps(frame, indent=1))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('out', type=Path)
    parser.add_argument('--scenes', type=int, default=50)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--params', type=Path, help='JSON overriding SCENE_PARAMS')
    opts = parser.parse_args()
    params = {**SCENE_PARAMS, **(json.loads(opts.params.read_text()) if opts.params else {})}
    unknown = set(params) - set(SCENE_PARAMS)
    if unknown:
        raise SystemExit(f'unknown params: {sorted(unknown)}')
    rng = random.Random(opts.seed)
    TMP.mkdir(exist_ok=True)

    sim = Sim()
    run_started, done, resumed, error = time.monotonic(), [], 0, None
    try:
        names = list(OBJECTS) + [f'distractor{i}' for i in range(len(DISTRACTORS))]
        sim.spin(2.0)
        if sim.pose(OBJECTS['beer']) is None:
            raise RuntimeError('no bottles in the world: is this workcell_world.sdf from this branch?')
        for i, (_, parts) in enumerate(DISTRACTORS):
            if sim.pose(f'distractor{i}') is None:
                spawn(f'distractor{i}', distractor_sdf(f'distractor{i}', parts, 20 + i))
        for i, bottle in enumerate(OBJECTS):
            if sim.pose(f'held_{bottle}') is None:
                spawn(f'held_{bottle}', held_sdf(bottle, i + 1))
        if sim.pose('vlm_camera') is None:
            spawn('vlm_camera', camera_sdf(), z=2.0)
        sim.spin(2.0)
        ids = visual_ids()

        for n in range(opts.scenes):
            started = time.monotonic()
            scene_dir = opts.out / f'w{opts.seed:03d}_{n:05d}'
            if (scene_dir / 'scene.json').exists():
                replay(rng, params)
                resumed += 1
                continue
            scene_dir.mkdir(parents=True, exist_ok=True)
            # Clear the table first, so bottles are never teleported into the arm.
            sim.move_arm(HOME, seconds=1.0, wait=2.0)
            layout = sample_layout(rng, params)
            held = sample_held(rng, params, layout)
            for i, name in enumerate(names):
                place(model_name(name), layout.get(name), 10 + i)
            for i, bottle in enumerate(OBJECTS):
                set_pose(f'held_{bottle}', 20 + i, 10, HIDDEN_Z)
            look, distractor_colours = sample_appearance(rng, params)
            apply_appearance(ids, look, distractor_colours)
            sim.spin(0.5)
            arm_kind, arm_target, joints = sample_arm(rng, params, layout, held)
            sim.grip(GRIP_CLOSED if held else GRIP_OPEN)
            arm_ok = sim.move_arm(joints)
            if held:
                sim.spin(1.0)  # the twin can still be settling; 0.3s read it mid-move
                wx, wy, wz = sim.pose('wrist_3_link')['xyz']  # relative to the table's corner
                if wz < HOLD_DROP + 0.05:
                    # The twin stuck short (pressed on the table or a bottle): a bottle
                    # hung from there would sit inside the table. Leave it out instead.
                    held, arm_kind, arm_target = None, 'over_table', None
                else:
                    set_pose(f'held_{held}', wx, wy, TABLE_Z + wz - HOLD_DROP, (0.0, 0.0, joints[0]))
            family, eye, rpy = sample_camera(rng, params)
            set_pose('vlm_camera', *eye, rpy)
            sim.spin(1.5)  # physics settles, the camera re-renders
            rgb, labels = sim.frame()
            objects = settled_objects(sim, {k: model_name(k) for k in names if k in layout})
            if held:
                objects[held] = {'on_table': False, 'in_gripper': True}
            for name, spot in layout.items():
                if 'kind' in spot:
                    objects[name]['kind'] = spot['kind']
            save(scene_dir, rgb, labels, {
                'in_gripper': held, 'objects': objects,
                'arm': {'kind': arm_kind, 'target': arm_target, 'reached': arm_ok, **arm_state(sim),
                        'commanded': [round(q, 3) for q in joints]}})
            (scene_dir / 'scene.json').write_text(json.dumps({
                'bottles': list(OBJECTS), 'glasses': [], 'kind': 'workcell',
                'seed': opts.seed, 'index': n, 'frames': 1,
                'seconds': round(time.monotonic() - started, 1),
                'camera': {'family': family, 'eye': [round(v, 3) for v in eye],
                           'rpy': [round(v, 4) for v in rpy], 'size': CAM_SIZE, 'hfov': CAM_HFOV},
                'appearance': {'randomized': look is not None, **(look or DEFAULT_LOOK),
                               'distractors': {k: v for k, v in distractor_colours.items() if k in layout}},
                'arm_base': ARM_BASE, 'table': TABLE,
                'params': {'layout': {k: {**v, 'xy': [round(c, 3) for c in v['xy']]} for k, v in layout.items()},
                           'scene_params': params}}, indent=1))
            done.append({'camera': family, 'arm': arm_kind, 'reached': arm_ok, 'held': held,
                         'seconds': time.monotonic() - started})
            print(f'{scene_dir.name} cam={family} arm={arm_kind}/{arm_ok} '
                  f'objects={sorted(layout)} {time.monotonic() - started:.1f}s', flush=True)
    except BaseException as e:
        error = f'{type(e).__name__}: {e}'
        raise
    finally:
        sim.close()
        opts.out.mkdir(parents=True, exist_ok=True)
        log_run(opts.out, opts, params, run_started, done, resumed, error)


if __name__ == '__main__':
    main()
