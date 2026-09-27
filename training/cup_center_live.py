#!/usr/bin/env python3
"""Live centre of the orange cup, by DINOv2 patch matching; serves an annotated MJPEG view.

    python3 cup_center_live.py --ref ref.jpg --ref-box 822 410 918 496

--ref/--ref-box (repeatable) enrol the cup from frames of this same camera,
with a tight box. Every frame after that, each DINOv2 patch is scored as
similarity to the cup minus similarity to the background of those frames; the
best-scoring blob's centroid is the cup centre.
No detector: off-the-shelf ones (YOLO11 COCO, Grounding DINO, OWLv2) missed
this cup top-down or ranked other objects above it.

View at http://<host>:8767/ ; GET /position returns the latest centre as JSON
{found, x, y, score, updated_at}, for a VLM/planner to poll for where to pour.
The centre is also printed every --report seconds.
"""
import argparse
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from transformers import AutoModel

PATCH = 14
# The best-matching patches are the ribbed wall the camera sees side-on, not the
# middle: on 4 hand-boxed frames (leave-one-out) the centroid sat 16-23px right
# of the true centre, spread 3px. Pixels of the 1280x720 cam0 frame.
# ponytail: one fixed offset for the whole table; re-measure per region if the
# camera or its angle changes, or a far corner shows a different bias.
CENTRE_OFFSET = (-20, 0)
CENTRE_FRAC = 0.4
BOX_SCALE = 1.2
MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


def mjpeg_frames(url, stall_timeout=5):
    """Yield frames from a multipart MJPEG stream; reconnect if reads stall.

    A read can block far longer than the caller expects on a proxied,
    irregular-chunk multipart stream (seen live: the generator silently froze
    on the last frame for 8+ minutes with no exception). socket timeout turns
    a silent hang into a retried connection instead.
    """
    while True:
        try:
            stream = urllib.request.urlopen(url, timeout=stall_timeout)
            buf = b''
            while True:
                chunk = stream.read(65536)
                if not chunk:
                    raise ConnectionError('stream closed')
                buf += chunk
                end = buf.rfind(b'\xff\xd9')
                if end == -1:
                    continue
                start = buf.rfind(b'\xff\xd8', 0, end)
                if start == -1:
                    continue
                # newest complete frame only, so a slow consumer never lags behind
                jpeg, buf = buf[start:end + 2], buf[end + 2:]
                frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    yield frame
        except (OSError, ConnectionError) as e:
            print(f'stream read failed ({e}), reconnecting in 1s', flush=True)
            time.sleep(1)


class PatchMatcher:
    def __init__(self, model_id, width, device):
        self.model = AutoModel.from_pretrained(model_id).to(device).half().eval()
        self.width = width // PATCH * PATCH
        self.device = device

    def features(self, bgr):
        """Unit-norm patch features, shape (rows, cols, dim), and the patch grid size in source px."""
        h, w = bgr.shape[:2]
        height = round(h * self.width / w) // PATCH * PATCH
        rgb = cv2.resize(bgr, (self.width, height))[:, :, ::-1].copy()
        x = (torch.from_numpy(rgb).permute(2, 0, 1).float() / 255 - MEAN) / STD
        with torch.no_grad():
            tokens = self.model(pixel_values=x[None].to(self.device).half()).last_hidden_state
        rows, cols = height // PATCH, self.width // PATCH
        patches = tokens[0, -rows * cols:].float().view(rows, cols, -1)
        return F.normalize(patches, dim=-1), (w / cols, h / rows)


def enrol(matcher, ref_boxes, pad=1):
    """Cup and background reference patches from (image_path, (x0,y0,x1,y1)) frames of this camera.

    Patches inside the tight cup box are the cup; everything outside it (grown
    by `pad` patches) is background -- table, cables, robot, the black button.
    Scoring against both is what keeps bare table from matching: with cup
    patches alone, loose boxes put table into the reference and the table
    out-scored the cup on every held-out frame.
    """
    cup, background = [], []
    for ref_path, (x0, y0, x1, y1) in ref_boxes:
        feats, (sx, sy) = matcher.features(cv2.imread(ref_path))
        rows, cols, _ = feats.shape
        r0, r1 = int(y0 / sy), int(np.ceil(y1 / sy))
        c0, c1 = int(x0 / sx), int(np.ceil(x1 / sx))
        inside = torch.zeros(rows, cols, dtype=torch.bool)
        inside[r0:r1, c0:c1] = True
        near = torch.zeros(rows, cols, dtype=torch.bool)
        near[max(r0 - pad, 0):r1 + pad, max(c0 - pad, 0):c1 + pad] = True
        cup.append(feats[inside])
        background.append(feats[~near])
    return torch.cat(cup), torch.cat(background)


def locate(matcher, frame, ref, min_score):
    """Cup centre and its matched-region box, both in source pixels, and the match score.

    score = similarity to the cup minus similarity to the background, so ~0 on
    anything that also appears in the background references. Returns
    (None, None, score) below min_score: cup hidden, out of view, or unsure.
    """
    cup, background = ref
    feats, (sx, sy) = matcher.features(frame)
    score = ((feats @ cup.T).max(dim=-1).values - (feats @ background.T).max(dim=-1).values).cpu().numpy()
    score = cv2.GaussianBlur(score, (3, 3), 0)
    peak = float(score.max())
    if peak < min_score:
        return None, None, peak
    _, labels = cv2.connectedComponents((score > peak * CENTRE_FRAC).astype(np.uint8))
    py, px = np.unravel_index(score.argmax(), score.shape)
    ys, xs = np.nonzero(labels == labels[py, px])
    w = score[ys, xs]
    cx = (np.average(xs, weights=w) + 0.5) * sx + CENTRE_OFFSET[0]
    cy = (np.average(ys, weights=w) + 0.5) * sy + CENTRE_OFFSET[1]
    # box size from the tighter core blob, centred on the corrected middle
    _, core = cv2.connectedComponents((score > peak * 0.6).astype(np.uint8))
    ys, xs = np.nonzero(core == core[py, px])
    hw, hh = (xs.max() + 1 - xs.min()) * sx * BOX_SCALE / 2, (ys.max() + 1 - ys.min()) * sy * BOX_SCALE / 2
    box = (cx - hw, cy - hh, cx + hw, cy + hh)
    return (cx, cy), box, peak


class Pipeline:
    def __init__(self, opts):
        self.lock = threading.Lock()
        self.jpeg = None
        self.position = {'found': False, 'x': None, 'y': None, 'score': None, 'updated_at': None}
        self.opts = opts
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        o = self.opts
        matcher = PatchMatcher(o.model, o.width, 'cuda')
        ref = enrol(matcher, zip(o.ref, o.ref_box))
        print(f'enrolled {len(ref[0])} cup / {len(ref[1])} background patches '
              f'from {len(o.ref)} frame(s)', flush=True)
        smooth, last_report, fps = None, 0.0, 0.0
        for frame in mjpeg_frames(o.source):
            t = time.time()
            centre, box, score = locate(matcher, frame, ref, o.min_score)
            if centre is not None:
                smooth = centre if smooth is None else tuple(
                    o.alpha * c + (1 - o.alpha) * s for c, s in zip(centre, smooth))
                cx, cy = int(smooth[0]), int(smooth[1])
                x0, y0, x1, y1 = (int(v) for v in box)
                cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 0, 255), 3)
                cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 40, 3)
                cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)
                cv2.putText(frame, f'cup ({cx},{cy}) score {score:.2f}', (cx + 25, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                smooth = None
                cv2.putText(frame, f'CUP NOT FOUND (best score {score:.2f})', (30, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            fps = 0.9 * fps + 0.1 / max(time.time() - t, 1e-3)
            cv2.putText(frame, f'{fps:.1f} fps (GPU)', (30, frame.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if time.time() - last_report > o.report:
                print(f'{time.strftime("%H:%M:%S")} centre={smooth and tuple(int(v) for v in smooth)} '
                      f'score={score:.2f} fps={fps:.1f}', flush=True)
                last_report = time.time()
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            position = {'found': centre is not None,
                        'x': int(smooth[0]) if smooth else None, 'y': int(smooth[1]) if smooth else None,
                        'score': round(score, 3), 'updated_at': time.time()}
            with self.lock:
                if ok:
                    self.jpeg = buf.tobytes()
                self.position = position

    def latest(self):
        with self.lock:
            return self.jpeg

    def latest_position(self):
        with self.lock:
            return dict(self.position)


def make_handler(pipeline):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == '/':
                body = b'<html><body style="margin:0;background:#111"><img src="/stream" style="width:100%"></body></html>'
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == '/position':
                body = json.dumps(pipeline.latest_position()).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != '/stream':
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    jpeg = pipeline.latest()
                    if jpeg is not None:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--source', default='http://localhost:8765/cam0_upside')
    p.add_argument('--ref', action='append', required=True,
                   help='a frame from the same camera showing the cup; repeat with --ref-box for more crops')
    p.add_argument('--ref-box', type=int, nargs=4, action='append', required=True,
                   metavar=('X0', 'Y0', 'X1', 'Y1'))
    p.add_argument('--model', default='facebook/dinov2-small')
    p.add_argument('--width', type=int, default=896, help='DINOv2 input width')
    p.add_argument('--min-score', type=float, default=0.08,
                   help='cup-minus-background score below which the cup counts as not found')
    p.add_argument('--alpha', type=float, default=0.5, help='smoothing: 1 = no smoothing')
    p.add_argument('--report', type=float, default=10)
    p.add_argument('--port', type=int, default=8767)
    o = p.parse_args()
    pipeline = Pipeline(o)
    ThreadingHTTPServer(('0.0.0.0', o.port), make_handler(pipeline)).serve_forever()


if __name__ == '__main__':
    main()
