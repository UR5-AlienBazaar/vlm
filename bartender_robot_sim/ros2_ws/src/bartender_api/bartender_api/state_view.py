"""GET /state: joints, tool pose, gripper, per arm -- read-only.

Reuses bartender_teach.teach_points.TeachNode exactly as teach_gui.py does,
rather than re-subscribing to /joint_states and re-implementing FK: the
pendant already owns this reading, and a second implementation is a second
set of bugs -- the same principle CONTROL_API.md states for movement
("wrap Pendant.dispatch(), do not reimplement it"), generalised here to
reading state instead of commanding it.
"""
from bartender_teach.teach_points import ARMS


def build(node):
    """Assemble the /state document from a live TeachNode.

    `node.cached_pose(arm)` can block on a fresh FK call if its 250ms timer
    cache does not hold this arm's pose yet (see TeachNode._refresh_pose);
    that is the same cost teach_gui already pays and is fine for a
    diagnostic endpoint, not one meant to be polled at motion-control rates.
    """
    joints = node.joints()
    arms = []
    for arm in ARMS.values():
        pose = node.cached_pose(arm)
        arms.append({
            'id': arm.key,
            'label': arm.label,
            'frame': arm.frame,
            'connected': any(j in joints for j in arm.joints),
            'joints': {j: joints[j] for j in arm.joints if j in joints},
            'pose': None if pose is None else {
                'xyz': list(pose[0]), 'quat_xyzw': list(pose[1]),
            },
            'gripper': joints.get(arm.gripper_joint),
        })
    return {'arms': arms}
