#!/usr/bin/env python3
"""Capture randomized bar scenes from the running sim for the VLM dataset (see vlm_labels.py).

Run inside the sim container while bartender_sim.launch.py is up:
    docker exec bartender-robot bash -lc \\
        "source ros2_ws/install/setup.bash && python3 scripts/capture_vlm_frames.py data/vlm_raw --scenes 400"
"""
import argparse
import json
import math
import random
import subprocess
import time
from pathlib import Path

import numpy as np

WORLD = 'bar_world'
CAMERAS = {
    'overhead': '/bartender/overhead_camera',
    'stand': '/bartender/stand_camera',
    'wrist': '/bartender/arm_a/wrist_camera',
}
RGB_TOPIC = {'overhead': 'image_raw', 'stand': 'rgb', 'wrist': 'image_raw'}
MODELS = {'whiskey': 'jack_daniels_bottle', 'cola': 'cola_bottle', 'beer': 'beer_bottle'}
# Stand centres from bar_world.sdf. Bottles only rotate in place: each sits in
# a 26mm-deep stand recess, so a sideways nudge would tip it.
STANDS = {'whiskey': (0.08, -0.30), 'cola': (0.08, -0.15)}
GLASS_HOME = (0.20, -0.55)
# The glass has open counter around it; 5cm keeps it clear of the whiskey
# stand (0.25m away) and inside the overhead camera's view.
GLASS_JITTER = 0.05
DISTRACTOR_P = 0.30
POUR_P = 0.40
# PourDrink feedback states "<phase>_<bottle>" in which that bottle is in the
# gripper: from the lift off its stand until it is set back down. Taken from the
# server's own state machine rather than guessed from finger angle and height,
# which labelled 1 of ~60 held frames in the first trial.
HELD_PHASES = {'lifting', 'moving_to_glass', 'tilting_to_pour', 'pouring',
               'checking_grip', 'returning_upright', 'returning', 'lowering'}
# A bottle leaning more than this at the end of a pour fell over (the scripted
# place can land it on the stand's rim); such scenes are flagged, not dropped.
UPRIGHT_MAX_TILT_DEG = 20
HIDDEN = (5.0, 0.0, 0.2)
DISTRACTOR_SDF = '/tmp/vlm_distractor.sdf'


def held_bottle(state):
    """Bottle in the gripper for a PourDrink feedback state, e.g. 'pouring_whiskey'."""
    phase, _, bottle = state.rpartition('_')
    # 'returning_home' parses as phase 'returning' too; only real bottles count.
    return bottle if phase in HELD_PHASES and bottle in MODELS else None


def tilt_deg(q):
    """Angle between a model's up axis and the world's, from its orientation quaternion."""
    up_z = 1 - 2 * (q.x ** 2 + q.y ** 2)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def sample_scene(rng):
    """Random static scene: which bottles show, their yaw, glass offset, distractor."""
    shown = [b for b in STANDS if rng.random() < 0.7]
    distractor = None
    if rng.random() < DISTRACTOR_P:
        # Half near a target (obstruction positives), half anywhere on arm A's side.
        tx, ty = rng.choice([*[STANDS[b] for b in shown], GLASS_HOME])
        near = rng.random() < 0.5
        distractor = ((tx - rng.uniform(0.06, 0.12), ty + rng.uniform(-0.05, 0.05)) if near
                      else (rng.uniform(-0.25, 0.25), rng.uniform(-0.75, 0.2)))
    return {
        'shown': shown,
        'yaw': {b: rng.uniform(-math.pi, math.pi) for b in shown},
        'glass': (GLASS_HOME[0] + rng.uniform(-GLASS_JITTER, GLASS_JITTER),
                  GLASS_HOME[1] + rng.uniform(-GLASS_JITTER, GLASS_JITTER)),
        'distractor': distractor,
    }


def _ign(service, reqtype, req, reptype='ignition.msgs.Boolean'):
    subprocess.run(['ign', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
                    '--reptype', reptype, '--timeout', '3000', '--req', req],
                   check=True, capture_output=True)


def set_pose(model, x, y, z, yaw=0.0):
    _ign('set_pose', 'ignition.msgs.Pose',
         f'name: "{model}" position {{x: {x} y: {y} z: {z}}} '
         f'orientation {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')


def spawn_distractor(x, y):
    Path(DISTRACTOR_SDF).write_text(
        '<sdf version="1.9"><model name="distractor"><link name="link">'
        '<inertial><mass>0.2</mass></inertial>'
        '<collision name="c"><geometry><box><size>0.06 0.06 0.18</size></box></geometry></collision>'
        '<visual name="v"><geometry><box><size>0.06 0.06 0.18</size></box></geometry>'
        '<material><diffuse>0.2 0.4 0.8 1</diffuse></material></visual></link>'
        '<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>20</label></plugin>'
        '</model></sdf>')
    _ign('create', 'ignition.msgs.EntityFactory',
         f'sdf_filename: "{DISTRACTOR_SDF}" pose {{position {{x: {x} y: {y} z: 0.99}}}}')


def remove_distractor():
    try:
        _ign('remove', 'ignition.msgs.Entity', 'name: "distractor" type: MODEL')
    except subprocess.CalledProcessError:
        pass  # nothing spawned in this scene


class Capture:
    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from sensor_msgs.msg import Image
        from tf2_msgs.msg import TFMessage
        from bartender_pour_interfaces.action import PourDrink

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('capture_vlm_frames')
        self.latest = {}
        self.poses = {}
        self.state = ''
        self.phases = []
        for cam, base in CAMERAS.items():
            for kind, topic in (('rgb', f'{base}/{RGB_TOPIC[cam]}'),
                                ('labels', f'{base}/segmentation/labels_map')):
                self.node.create_subscription(
                    Image, topic, lambda m, k=(cam, kind): self.latest.__setitem__(k, m), 2)
        self.node.create_subscription(
            TFMessage, f'/world/{WORLD}/dynamic_pose/info',
            lambda m: self.poses.update({t.child_frame_id: t.transform for t in m.transforms}), 2)
        self.pour = ActionClient(self.node, PourDrink, 'pour_drink')
        self.PourDrink = PourDrink

    def on_feedback(self, msg):
        if msg.feedback.state != self.state:
            self.state = msg.feedback.state
            self.phases.append((round(time.monotonic() - self.t0, 1), self.state))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def frames(self, timeout=10.0):
        """Fresh RGB + label pairs rendered on the same tick, one per camera."""
        self.latest.clear()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            pairs = {c: (self.latest.get((c, 'rgb')), self.latest.get((c, 'labels'))) for c in CAMERAS}
            if all(r and l and r.header.stamp == l.header.stamp for r, l in pairs.values()):
                return pairs
        raise TimeoutError('cameras did not deliver matching frames')

    def upright(self):
        return {b: tilt_deg(self.poses[m].rotation) < UPRIGHT_MAX_TILT_DEG
                for b, m in MODELS.items() if m in self.poses}


def save(scene_dir, index, pairs, in_gripper):
    from PIL import Image as PILImage
    for cam, (rgb, labels) in pairs.items():
        img = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, -1)
        lab = np.frombuffer(labels.data, np.uint8).reshape(labels.height, labels.width, -1)[:, :, 0]
        PILImage.fromarray(img[:, :, :3]).save(scene_dir / f'{index:04d}_{cam}_rgb.png')
        PILImage.fromarray(lab).save(scene_dir / f'{index:04d}_{cam}_labels.png')
    (scene_dir / f'{index:04d}.json').write_text(json.dumps({'in_gripper': in_gripper}))


def reset(shown=('whiskey', 'cola'), yaw=None, glass=GLASS_HOME):
    for bottle, (x, y) in STANDS.items():
        if bottle in shown:
            set_pose(MODELS[bottle], x, y, 0.9, (yaw or {}).get(bottle, 0.0))
        else:
            set_pose(MODELS[bottle], *HIDDEN)
    set_pose('serving_glass', *glass, 0.9)
    remove_distractor()


def static_scene(cap, scene_dir, rng):
    scene = sample_scene(rng)
    reset(scene['shown'], scene['yaw'], scene['glass'])
    if scene['distractor']:
        spawn_distractor(*scene['distractor'])
    cap.spin(1.5)  # let the physics settle before rendering
    save(scene_dir, 0, cap.frames(), None)
    return [*scene['shown'], 'beer'], {'params': scene}


def pour_scene(cap, scene_dir, max_frames=300):
    """Run the scripted pour (whiskey then cola, from their home spots) and film it.

    At the sim's ~1.7 fps software rendering the whole recipe takes ~200 frames.
    """
    reset()
    cap.spin(1.5)
    cap.state, cap.phases, cap.t0 = '', [], time.monotonic()
    goal = cap.PourDrink.Goal(bottle_id='whiskey', glass_id='serving_glass', pour_amount_ml=40.0)
    cap.pour.wait_for_server(timeout_sec=60)
    accepted = cap.pour.send_goal_async(goal, feedback_callback=cap.on_feedback)
    finished = None
    for i in range(max_frames):
        pairs = cap.frames()
        save(scene_dir, i, pairs, held_bottle(cap.state))
        if finished is None and accepted.done():
            finished = accepted.result().get_result_async()
        if finished is not None and finished.done():
            break
        cap.spin(1.0)
    result = finished.result().result if finished is not None and finished.done() else None
    return ['whiskey', 'cola', 'beer'], {'pour': {
        'finished': result is not None,
        'success': bool(result and result.success),
        'message': result.message if result else 'still running at max_frames',
        'phases': cap.phases,
        'upright_after': cap.upright(),
    }}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('out', type=Path)
    parser.add_argument('--scenes', type=int, default=400)
    parser.add_argument('--seed', type=int, default=0)
    opts = parser.parse_args()
    rng = random.Random(opts.seed)
    cap = Capture()
    cap.spin(3.0)
    for n in range(opts.scenes):
        scene_dir = opts.out / f's{opts.seed:02d}_{n:05d}'
        scene_dir.mkdir(parents=True, exist_ok=True)
        pour = rng.random() < POUR_P
        started = time.monotonic()
        bottles, log = pour_scene(cap, scene_dir) if pour else static_scene(cap, scene_dir, rng)
        frames = len(list(scene_dir.glob('*_overhead_rgb.png')))
        (scene_dir / 'scene.json').write_text(json.dumps({
            'bottles': bottles, 'glasses': ['glass'], 'kind': 'pour' if pour else 'static',
            'seed': opts.seed, 'index': n, 'frames': frames,
            'seconds': round(time.monotonic() - started, 1), **log}, indent=1))
        print(f'{scene_dir.name} {"pour" if pour else "static"} {frames} frames '
              f'{json.dumps(log.get("pour", {}).get("upright_after", ""))}', flush=True)


if __name__ == '__main__':
    main()
