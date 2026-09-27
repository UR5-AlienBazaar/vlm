#!/usr/bin/env python3
"""Serve bottle boxes found by Gemini, in live_bottle_infer.py's /objects format.

Every --interval seconds the newest cam0 frame goes to Gemini, which returns a
box per known bottle. GET /objects returns the latest answer, GET /stream the
annotated frame. Needs GEMINI_API_KEY in the environment.
"""
import argparse
import base64
import json
import os
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

import cv2
import numpy as np

from bottle_vision.camera import frames
from bottle_vision.serve import make_handler
from training.live_bottle_infer import draw

# Labels match the DINOv2 classifier so consumers see one vocabulary.
BOTTLES = {
    "7up": "clear plastic bottle, green 7UP label",
    "mirinda": "plastic bottle of orange soda",
    "cola": "dark plastic cola bottle with a blue cap",
    "vodka": "clear glass bottle (Zubrowka vodka)",
    "liqueur": "green square glass bottle (Jagermeister)",
    # Ballantine's is a whisky too; without the top-down look spelled out it came back as "whiskey".
    "ballantines": "Ballantine's Scotch. From above: round dark bottle with a silver/white cap, sitting inside a black square box",
    "whiskey": "Jack Daniel's. From above: square clear glass bottle, no box around it",
}
PROMPT = (
    "Top-down photo of a bar table. Find every bottle from this list and return a JSON array of "
    '{"label": <one key below>, "box_2d": [ymin, xmin, ymax, xmax] normalised to 0-1000}. '
    "Omit bottles that are not visible; do not return anything that is not a bottle.\n"
    + "\n".join(f"{label}: {description}" for label, description in BOTTLES.items())
)


def ask_gemini(jpeg, model, key):
    body = {"contents": [{"parts": [{"inline_data": {"mime_type": "image/jpeg", "data": base64.b64encode(jpeg).decode()}},
                                    {"text": PROMPT}]}],
            "generationConfig": {"responseMimeType": "application/json", "temperature": 0}}
    request = urllib.request.Request(f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent",
                                     json.dumps(body).encode(), {"Content-Type": "application/json", "x-goog-api-key": key})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)["candidates"][0]["content"]["parts"][0]["text"]


def to_objects(answer, width, height):
    """Gemini's JSON answer -> /objects rows in full-frame pixels; unknown labels and bad boxes are dropped."""
    objects = []
    # The model sometimes appends a second array after the first; only the first is the answer.
    for index, item in enumerate(json.JSONDecoder().raw_decode(answer.lstrip())[0]):
        box = item.get("box_2d")
        if item.get("label") not in BOTTLES or not isinstance(box, list) or len(box) != 4:
            continue
        # [ymin, xmin, ymax, xmax] is the robotics-ER model's native format; gemini-2.5-flash
        # sometimes returned [x, y, x, y] instead, so it is not a supported --model.
        y0, x0, y1, x1 = (round(v / 1000 * size) for v, size in zip(box, (height, width, height, width)))
        objects.append({"track_id": index + 1, "label": item["label"], "best_guess": item["label"], "score": None,
                        "bbox": [x0, y0, x1, y1], "x": (x0 + x1) // 2, "y": (y0 + y1) // 2, "found": True, "source": "gemini"})
    return objects


class Pipeline:
    def __init__(self, args):
        self.args, self.lock, self.jpeg, self.objects = args, threading.Lock(), None, []
        threading.Thread(target=self.loop, daemon=True).start()

    def image(self):
        with self.lock: return self.jpeg

    def state(self):
        with self.lock: return list(self.objects)

    def position(self):
        return {"error": "cup is served by live_bottle_infer.py"}

    def loop(self):
        key, last_call, answered = os.environ["GEMINI_API_KEY"], 0.0, None
        for frame in frames(self.args.source):
            if time.time() - last_call < self.args.interval:
                continue
            if frame.shape[1] != self.args.frame_width:  # keep pixels comparable with live_bottle_infer.py
                frame = cv2.resize(frame, (self.args.frame_width, round(frame.shape[0] * self.args.frame_width / frame.shape[1])))
            thumb = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 90), interpolation=cv2.INTER_AREA).astype(np.int16)
            change = float(np.abs(thumb - answered).mean()) if answered is not None else float("inf")
            # The cached answer stays valid while the table looks the same; --max-age bounds
            # how long a change too small to cross --change can go unnoticed.
            if change < self.args.change and time.time() - last_call < self.args.max_age:
                continue
            last_call = time.time()
            try:
                objects = to_objects(ask_gemini(cv2.imencode(".jpg", frame)[1].tobytes(), self.args.model, key), frame.shape[1], frame.shape[0])
            except (OSError, ValueError, KeyError, IndexError) as error:
                print(f"gemini failed: {error}", flush=True)
                # The table changed since the cached answer, so serving it would be wrong.
                with self.lock: self.objects = []
                continue
            answered = thumb
            print(f"gemini call: change {change:.1f}, {len(objects)} bottles", flush=True)
            for row in objects:
                draw(frame, row["track_id"], row["label"], 1.0, row["bbox"], 3)
            with self.lock:
                self.objects, self.jpeg = objects, cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="http://localhost:8778/cam0_upside")
    parser.add_argument("--model", default="gemini-robotics-er-2-preview")
    parser.add_argument("--interval", type=float, default=2.0, help="minimum seconds between Gemini calls")
    parser.add_argument("--change", type=float, default=3.0, help="mean grey-level change (0-255) vs the last answered frame that triggers a new call")
    parser.add_argument("--max-age", type=float, default=60.0, help="re-ask after this many seconds even if nothing changed")
    parser.add_argument("--frame-width", type=int, default=1280)
    parser.add_argument("--port", type=int, default=8780)
    args = parser.parse_args()
    ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(Pipeline(args))).serve_forever()


if __name__ == "__main__":
    main()
