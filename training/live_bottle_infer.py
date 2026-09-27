#!/usr/bin/env python3
"""Serve YOLO + ByteTrack boxes labelled by the trained DINOv2 head.

GET /stream is the annotated MJPEG; GET /objects returns each tracked bottle's
current box, pixel centre, smoothed label and score. Labels below --threshold
are reported as null rather than guessed.
"""
import argparse
from http.server import ThreadingHTTPServer
from pathlib import Path
import threading

import cv2
import numpy as np
import torch
from torchvision import transforms

from bottle_vision.camera import frames
from bottle_vision.classifier import DinoBottleClassifier
from bottle_vision.serve import make_handler
from bottle_vision.temporal import ProbabilitySmoother
from bottle_vision.tracker import YOLOByteTracker


# BGR, one distinct colour per drink; grey means "tracked but not sure what".
COLOURS = {"cola": (40, 40, 230), "mirinda": (0, 140, 255), "7up": (60, 200, 60), "liqueur": (30, 90, 30),
           "vodka": (255, 200, 80), "ballantines": (200, 60, 200)}
UNKNOWN_COLOUR = (160, 160, 160)


class Pipeline:
    def __init__(self, args):
        self.args, self.lock, self.jpeg, self.objects = args, threading.Lock(), None, []
        self.recent = {}  # track_id -> (frames since last seen, last /objects row)
        threading.Thread(target=self.loop, daemon=True).start()

    def image(self):
        with self.lock: return self.jpeg

    def state(self):
        with self.lock: return list(self.objects)

    def loop(self):
        args = self.args
        checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=True); self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.classes = checkpoint["classes"]
        self.model = DinoBottleClassifier(checkpoint["model_id"], self.classes).to(self.device).eval(); self.model.load_state_dict(checkpoint["model"])
        size = checkpoint["image_size"]
        self.preprocess = transforms.Compose([transforms.ToPILImage(), transforms.Resize(int(size * 1.14)), transforms.CenterCrop(size),
                                              transforms.ToTensor(), transforms.Normalize((.485, .456, .406), (.229, .224, .225))])
        tracker, self.smoother = YOLOByteTracker(args.weights, args.detect_conf, image_size=args.imgsz), ProbabilitySmoother(args.history)
        for frame in frames(args.source):
            tracks = [track for track in tracker.update(frame) if track.track_id is not None]
            self.smoother.retain({track.track_id for track in tracks} | set(self.recent))
            objects = self.hold(self.label(frame, tracks), frame)
            ok, encoded = cv2.imencode(".jpg", frame)
            with self.lock:
                if ok: self.jpeg = encoded.tobytes()
                self.objects = objects

    def label(self, frame, tracks):
        """Classify each tracked crop, draw it on `frame`, and return its /objects rows."""
        crops = [frame[max(0, y0):y1, max(0, x0):x1] for x0, y0, x1, y1 in (track.box for track in tracks)]
        tracks, crops = [t for t, c in zip(tracks, crops) if c.size], [c for c in crops if c.size]
        if not crops:
            return []
        batch = torch.stack([self.preprocess(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)) for crop in crops]).to(self.device)
        with torch.inference_mode():
            probabilities = torch.softmax(self.model(batch).float(), 1).cpu().numpy()
        objects = []
        for track, raw in zip(tracks, probabilities):
            smoothed = self.smoother.update(track.track_id, raw); index = int(np.argmax(smoothed)); score = float(smoothed[index])
            label = self.classes[index] if score >= self.args.threshold else None
            x0, y0, x1, y1 = track.box
            draw(frame, track.track_id, label, score, track.box, 3)
            objects.append({"track_id": track.track_id, "label": label, "best_guess": self.classes[index], "score": round(score, 3),
                            "bbox": [x0, y0, x1, y1], "x": (x0 + x1) // 2, "y": (y0 + y1) // 2,
                            "detector_score": round(track.confidence, 3), "found": True})
        return objects
    def hold(self, objects, frame):
        """Keep a briefly-missed track on screen for --hold frames instead of blinking it out."""
        seen = {row["track_id"] for row in objects}
        self.recent = {tid: (0, row) for tid, row in ((row["track_id"], row) for row in objects)} | {
            tid: (age + 1, row) for tid, (age, row) in self.recent.items() if tid not in seen and age < self.args.hold}
        held = [{**row, "found": False, "frames_missing": age} for age, row in self.recent.values() if age]
        for row in held:
            draw(frame, row["track_id"], row["label"], row["score"], row["bbox"], 1)
        return objects + held


def draw(frame, track_id, label, score, box, thickness):
    x0, y0, x1, y1 = box; colour = COLOURS.get(label, UNKNOWN_COLOUR)
    cv2.rectangle(frame, (x0, y0), (x1, y1), colour, thickness)
    cv2.putText(frame, f"#{track_id} {(label or 'unknown').upper()} {score:.0%}", (x0, max(24, y0 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, .65, colour, 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--source", default="http://localhost:8765/cam0_upside")
    parser.add_argument("--weights", default="yolo11x.pt", help="YOLO weights with a \"bottle\" class")
    # ByteTrack keeps existing tracks alive with low-score boxes; cutting them
    # here instead makes boxes blink whenever a bottle's score dips.
    parser.add_argument("--detect-conf", type=float, default=.1)
    parser.add_argument("--imgsz", type=int, default=1280, help="YOLO input size; 640 misses small overhead bottles")
    parser.add_argument("--threshold", type=float, default=.6, help="minimum smoothed class probability to report a label")
    parser.add_argument("--history", type=int, default=15, help="frames of per-track probability smoothing")
    parser.add_argument("--hold", type=int, default=15, help="frames to keep showing a track after the detector loses it")
    parser.add_argument("--port", type=int, default=8770)
    args = parser.parse_args()
    ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(Pipeline(args))).serve_forever()


if __name__ == "__main__": main()
