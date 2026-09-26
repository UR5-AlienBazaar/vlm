#!/usr/bin/env python3
"""Capture randomized workcell scenes from the Gazebo twin for the VLM dataset.

Run inside the sim container while the twin is up:
    ros2 launch bartender_bringup workcell_twin.launch.py use_fake_hardware:=true \\
        headless:=true headless_rendering:=true
    python3 scripts/capture_workcell_scenes.py data/workcell_raw --scenes 50 --seed 3

Each scene: the mock arm moves to a random pose (the twin copies it), bottles
and distractors are placed on the table, one camera is put at a
random viewpoint, and one RGB + segmentation pair is saved together with the
objects' settled poses. The layout matches vlm/training/vlm_labels.py:

    <out>/<scene>/scene.json    bottles, glasses, camera, params
    <out>/<scene>/0000.json     in_gripper, settled object poses, arm joints
    <out>/<scene>/0000_cam_rgb.png, 0000_cam_labels.png

--params takes a JSON file overriding SCENE_PARAMS, which is how the VLM
harness steers what gets generated towards the cases the model gets wrong.
"""
import argparse
import json
import math
import random
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
# Distractor labels are 20 + index: vlm_labels treats 20-29 as distractors,
# and separate labels keep two distractors from merging into one box.
DISTRACTORS = [
    ('box', '<box><size>0.07 0.07 0.16</size></box>', (0.2, 0.4, 0.8)),
    ('can', '<cylinder><radius>0.033</radius><length>0.12</length></cylinder>', (0.8, 0.1, 0.1)),
    ('carton', '<box><size>0.10 0.05 0.20</size></box>', (0.9, 0.85, 0.3)),
]

SCENE_PARAMS = {
    'bottle_p': 0.7,          # each bottle is on the table
    'fallen_p': 0.08,         # a present bottle lies on its side
    'distractors': [0.4, 0.4, 0.2],  # P(0, 1, 2 distractors)
    'block_p': 0.4,           # a distractor stands between the arm and a bottle
    'arm_pose': {'home': 0.3, 'over_table': 0.7},
    'camera': {'front': 0.35, 'side': 0.25, 'overhead': 0.2, 'corner': 0.2},
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
    bottles = [b for b in ('whiskey', 'cola', 'beer') if b in placed]
    n = rng.choices(range(len(params['distractors'])), params['distractors'])[0]
    for i in range(n):
        spot = None
        if i == 0 and bottles and rng.random() < params['block_p']:
            bx, by = placed[rng.choice(bottles)]['xy']
            ax, ay = ARM_BASE
            d = math.dist((ax, ay), (bx, by))
            back = rng.uniform(0.09, 0.14) / d  # just in front of it, on the arm's side
            spot = (bx + (ax - bx) * back + rng.uniform(-0.02, 0.02),
                    by + (ay - by) * back + rng.uniform(-0.02, 0.02))
        placed[f'distractor{i}'] = {'xy': spot or free_spot() or (1.2, 0.1),
                                    'yaw': rng.uniform(-math.pi, math.pi), 'fallen': False}
    return placed


def sample_arm(rng, params):
    kind = _choice(rng, params['arm_pose'])
    if kind == 'home':
        return kind, HOME
    # Pan swept over the table's side of the arm, elbow bent down towards it.
    # Unconstrained joints drove the twin through the table and it never arrived.
    return kind, [rng.uniform(-2.6, -0.5), rng.uniform(-2.0, -1.0), rng.uniform(0.8, 2.0),
                  rng.uniform(-2.2, -1.0), rng.uniform(-1.8, -1.3), rng.uniform(-math.pi, math.pi)]


def sample_camera(rng, params):
    family = _choice(rng, params['camera'])
    centre, jitter, target = CAMERAS[family]
    eye = tuple(c + rng.uniform(-j, j) for c, j in zip(centre, jitter))
    target = (target[0] + rng.uniform(-0.1, 0.1), target[1] + rng.uniform(-0.08, 0.08), target[2])
    return family, eye, look_at(eye, target)


def _ign(service, reqtype, req, reptype='ignition.msgs.Boolean'):
    subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
                    '--reptype', reptype, '--timeout', '5000', '--req', req],
                   check=True, capture_output=True)


def set_pose(model, x, y, z, rpy=(0.0, 0.0, 0.0)):
    qx, qy, qz, qw = quat(*rpy)
    _ign('set_pose', 'ignition.msgs.Pose',
         f'name: "{model}" position {{x: {x} y: {y} z: {z}}} '
         f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}')


def spawn(name, sdf, z=HIDDEN_Z):
    path = TMP / f'{name}.sdf'
    path.write_text(sdf)
    _ign('create', 'ignition.msgs.EntityFactory',
         f'sdf_filename: "{path}" name: "{name}" allow_renaming: false '
         f'pose {{position {{x: 0 y: 0 z: {z}}}}}')


LABEL_PLUGIN = '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>{}</label></plugin>'


def distractor_sdf(name, geometry, rgb, label):
    color = ' '.join(map(str, rgb))
    return (f'<sdf version="1.9"><model name="{name}"><link name="link">'
            '<inertial><mass>0.2</mass></inertial>'
            f'<collision name="c"><geometry>{geometry}</geometry></collision>'
            f'<visual name="v"><geometry>{geometry}</geometry>'
            f'<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>'
            f'</link>{LABEL_PLUGIN.format(label)}</model></sdf>')


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
        from control_msgs.action import FollowJointTrajectory
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

    def close(self):
        self.bridge.terminate()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def move_arm(self, joints, seconds=2.0):
        """Move the mock robot; twin_mirror makes the Gazebo arm follow."""
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
        # The twin trails the mock by the mirror's latency; wait until it arrives.
        end = time.monotonic() + 8
        while time.monotonic() < end:
            if all(abs(self.joints.get(j, 1e9) - q) < 0.03 for j, q in zip(ARM_JOINTS, joints)):
                return True
            self.spin(0.1)
        return False

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
    rng = random.Random(opts.seed)
    TMP.mkdir(exist_ok=True)

    sim = Sim()
    try:
        names = list(OBJECTS) + [f'distractor{i}' for i in range(len(DISTRACTORS))]
        sim.spin(2.0)
        if sim.pose(OBJECTS['beer']) is None:
            raise RuntimeError('no bottles in the world: is this workcell_world.sdf from this branch?')
        for i, (_, geometry, rgb) in enumerate(DISTRACTORS):
            if sim.pose(f'distractor{i}') is None:
                spawn(f'distractor{i}', distractor_sdf(f'distractor{i}', geometry, rgb, 20 + i))
        if sim.pose('vlm_camera') is None:
            spawn('vlm_camera', camera_sdf(), z=2.0)
        sim.spin(2.0)

        for n in range(opts.scenes):
            started = time.monotonic()
            scene_dir = opts.out / f'w{opts.seed:03d}_{n:05d}'
            scene_dir.mkdir(parents=True, exist_ok=True)
            arm_kind, joints = sample_arm(rng, params)
            arm_ok = sim.move_arm(joints)
            layout = sample_layout(rng, params)
            for i, name in enumerate(names):
                place(model_name(name), layout.get(name), 10 + i)
            family, eye, rpy = sample_camera(rng, params)
            set_pose('vlm_camera', *eye, rpy)
            sim.spin(1.5)  # physics settles, the camera re-renders
            rgb, labels = sim.frame()
            objects = settled_objects(sim, {n: model_name(n) for n in names if n in layout})
            save(scene_dir, rgb, labels, {
                'in_gripper': None, 'objects': objects,
                'arm': {'kind': arm_kind, 'joints': [round(q, 3) for q in joints], 'reached': arm_ok}})
            (scene_dir / 'scene.json').write_text(json.dumps({
                'bottles': list(OBJECTS), 'glasses': [], 'kind': 'workcell',
                'seed': opts.seed, 'index': n, 'frames': 1,
                'seconds': round(time.monotonic() - started, 1),
                'camera': {'family': family, 'eye': [round(v, 3) for v in eye],
                           'rpy': [round(v, 4) for v in rpy], 'size': CAM_SIZE, 'hfov': CAM_HFOV},
                'arm_base': ARM_BASE, 'table': TABLE,
                'params': {'layout': {k: {**v, 'xy': [round(c, 3) for c in v['xy']]} for k, v in layout.items()},
                           'scene_params': params}}, indent=1))
            print(f'{scene_dir.name} cam={family} arm={arm_kind}/{arm_ok} '
                  f'objects={sorted(layout)} {time.monotonic() - started:.1f}s', flush=True)
    finally:
        sim.close()


if __name__ == '__main__':
    main()
