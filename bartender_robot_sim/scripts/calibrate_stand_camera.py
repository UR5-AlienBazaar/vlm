#!/usr/bin/env python3
"""Write stand_camera.yaml from counter ArUco markers; needs OAK-D depth aligned to RGB."""
import argparse
import sys

import cv2
import numpy as np

COUNTER_Z = 0.9


def marker_world_corners(x, y, size):
    """Return world corners in ArUco order (TL, TR, BR, BL); marker x is world -y."""
    h = size / 2.0
    return [(x + my, y - mx, COUNTER_Z)
            for mx, my in ((-h, h), (h, h), (h, -h), (-h, -h))]


def parse_marker(text):
    ident, x, y = text.split(':')
    return int(ident), float(x), float(y)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('image')
    parser.add_argument('--k', nargs=4, type=float, required=True,
                        metavar=('FX', 'FY', 'CX', 'CY'))
    parser.add_argument('--size', type=float, required=True,
                        help='marker edge length in metres')
    parser.add_argument('--marker', action='append', type=parse_marker,
                        required=True, help='id:x:y of a marker centre')
    parser.add_argument('--frame-id', required=True,
                        help='CameraInfo.header.frame_id of the aligned depth')
    parser.add_argument('--depth-range', nargs=2, type=float,
                        default=(0.35, 5.0))
    parser.add_argument('-o', '--output', required=True)
    opts = parser.parse_args()

    image = cv2.imread(opts.image, cv2.IMREAD_GRAYSCALE)
    if image is None:
        sys.exit(f'cannot read {opts.image}')
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = detector.detectMarkers(image)
    seen = {} if ids is None else {
        int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}

    world, pixels = [], []
    for ident, x, y in opts.marker:
        if ident not in seen:
            sys.exit(f'marker {ident} not found in {opts.image} '
                     f'(saw {sorted(seen)})')
        world += marker_world_corners(x, y, opts.size)
        pixels += seen[ident].tolist()
    world = np.array(world, dtype=np.float64)
    pixels = np.array(pixels, dtype=np.float64)

    fx, fy, cx, cy = opts.k
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(world, pixels, K, None,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        sys.exit('solvePnP failed')
    projected, _ = cv2.projectPoints(world, rvec, tvec, K, None)
    err = np.linalg.norm(projected.reshape(-1, 2) - pixels, axis=1)

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R.T
    T[:3, 3] = (-R.T @ tvec).flatten()

    rows = '\n'.join(f'  - [{", ".join(f"{v:.6f}" for v in row)}]'
                     for row in T)
    with open(opts.output, 'w', encoding='utf-8') as f:
        f.write(
            '# Stand camera optical-frame pose in world (maps optical -> world).\n'
            f'# HARDWARE, from {opts.image} by calibrate_stand_camera.py:\n'
            f'# {len(opts.marker)} marker(s), reprojection error mean '
            f'{err.mean():.2f} px, max {err.max():.2f} px.\n'
            f'optical_frame_id: {opts.frame_id}\n'
            f'T_world_optical:\n{rows}\n'
            f'depth_range: [{opts.depth_range[0]}, {opts.depth_range[1]}]\n')

    print(f'camera at world {T[:3, 3].round(4).tolist()}, looking along '
          f'{T[:3, 2].round(3).tolist()}')
    print(f'reprojection error: mean {err.mean():.2f} px, '
          f'max {err.max():.2f} px')
    print(f'wrote {opts.output}')


if __name__ == '__main__':
    main()
