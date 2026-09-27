#!/usr/bin/env python3
"""DINOv2 patch-match bottle tracker for the live overhead camera.

Unlike bottle_overlay_live.py, boxes are not fixed: every frame is scanned for
each enrolled bottle. A missing or uncertain bottle is shown as NOT FOUND.
"""
import argparse
import threading
from http.server import ThreadingHTTPServer

import cv2
import numpy as np
import torch.nn.functional as F

from bottle_vision.serve import make_handler
from cup_center_live import PatchMatcher, mjpeg_frames


def parse_reference(value):
    try:
        label, path, coordinates = value.split(":", 2)
        return label, path, tuple(map(int, coordinates.split(",")))
    except ValueError as error:
        raise argparse.ArgumentTypeError("--ref must be LABEL:IMAGE:X0,Y0,X1,Y1") from error


def enrol(matcher, references):
    enrolled = {}
    for label, path, box in references:
        image = cv2.imread(path)
        if image is None: raise FileNotFoundError(path)
        features, (sx, sy) = matcher.features(image); x0, y0, x1, y1 = box
        r0, r1, c0, c1 = int(y0 / sy), int(np.ceil(y1 / sy)), int(x0 / sx), int(np.ceil(x1 / sx))
        mask = np.zeros(features.shape[:2], bool); mask[r0:r1, c0:c1] = True
        near = np.zeros_like(mask); near[max(0,r0-1):r1+1, max(0,c0-1):c1+1] = True
        # One class/background prototype keeps six-object live matching cheap:
        # full patch-to-patch maxima made every frame six times heavier than the
        # already-proven single-target cup tracker.
        enrolled[label] = (F.normalize(features[mask].mean(0), dim=0),
                           F.normalize(features[~near].mean(0), dim=0), box)
    return enrolled


def locate(matcher, frame, reference, min_score, box_scale=.8):
    target, background, ref_box = reference
    features, (sx, sy) = matcher.features(frame)
    scores = (features @ target - features @ background).cpu().numpy()
    scores = cv2.GaussianBlur(scores, (3, 3), 0); peak = float(scores.max())
    if peak < min_score: return None, peak
    row, col = np.unravel_index(scores.argmax(), scores.shape)
    cx, cy = (col + .5) * sx, (row + .5) * sy
    width, height = (ref_box[2] - ref_box[0]) * box_scale, (ref_box[3] - ref_box[1]) * box_scale
    return (int(cx - width / 2), int(cy - height / 2), int(cx + width / 2), int(cy + height / 2)), peak


def iou(a, b):
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x1 - x0) * max(0, y1 - y0)
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union else 0.


class Pipeline:
    def __init__(self, source, refs, min_score):
        self.source, self.refs, self.min_score, self.lock, self.jpeg, self.objects = source, refs, min_score, threading.Lock(), None, []
        threading.Thread(target=self.loop, daemon=True).start()
    def loop(self):
        matcher = PatchMatcher("facebook/dinov2-small", 896, "cuda")
        references = enrol(matcher, self.refs)
        for frame in mjpeg_frames(self.source):
            output, objects, candidates = frame.copy(), [], []
            for label, reference in references.items():
                box, score = locate(matcher, frame, reference, self.min_score)
                if box: candidates.append((score, label, box))
            kept = []
            for score, label, box in sorted(candidates, reverse=True):
                if all(iou(box, other[2]) < .25 for other in kept): kept.append((score, label, box))
            matches = {label: (box, score) for score, label, box in kept}
            for label in references:
                box, score = matches.get(label, (None, 0.0))
                colour = (0, 0, 255) if label == "ballantines" else (0, 220, 0)
                if box:
                    x0,y0,x1,y1 = box; text = "BALLANTINE'S" if label == "ballantines" else label.upper()
                    cv2.rectangle(output,(x0,y0),(x1,y1),colour,3); cv2.putText(output,f"{text} {score:.2f}",(x0,max(26,y0-8)),cv2.FONT_HERSHEY_SIMPLEX,.65,colour,2)
                    objects.append({"label": label,"distractor":False,"bbox":[x0,y0,x1,y1],"x":(x0+x1)//2,"y":(y0+y1)//2,"score":round(score,3),"found":True})
                else: objects.append({"label":label,"distractor":False,"bbox":None,"x":None,"y":None,"score":round(score,3),"found":False})
            ok, encoded = cv2.imencode(".jpg", output)
            with self.lock:
                if ok: self.jpeg = encoded.tobytes()
                self.objects = objects
    def image(self):
        with self.lock: return self.jpeg
    def state(self):
        with self.lock: return list(self.objects)


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("--source",default="http://localhost:8765/cam0_upside"); p.add_argument("--ref",action="append",type=parse_reference,required=True); p.add_argument("--min-score",type=float,default=.06); p.add_argument("--port",type=int,default=8769); o=p.parse_args()
    ThreadingHTTPServer(("127.0.0.1",o.port),make_handler(Pipeline(o.source,o.ref,o.min_score))).serve_forever()
if __name__=="__main__": main()
