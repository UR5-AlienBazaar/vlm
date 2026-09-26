#!/usr/bin/env python3
"""Send live camera frames to a vLLM OpenAI endpoint and show what the scene VLM answers, with latency.

    python stream_vlm.py --source http://localhost:8090/view --endpoint http://localhost:8100
    python stream_vlm.py --source 0 --show            # webcam
    python stream_vlm.py --source rtsp://cam/stream --log stream.jsonl

--source is anything cv2.VideoCapture opens: a webcam index, MJPEG or RTSP URL, or a video file.
"""
import argparse
import base64
import json
import threading
import time

import cv2
import requests

from prelabel_real import GLASS, REAL_BAR_PROMPT, canonical_label


class LatestFrame:
    """Keeps only the newest frame: a network stream buffers frames while a request is in flight,
    and reading them in order would make every answer describe the past."""

    def __init__(self, source):
        self.capture = cv2.VideoCapture(int(source) if source.isdigit() else source)
        if not self.capture.isOpened():
            raise SystemExit(f'cannot open {source}')
        self.frame = None
        self.lock = threading.Lock()
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        while True:
            ok, frame = self.capture.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = frame

    def get(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def ask(endpoint, model, frame, prompt, timeout):
    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError('JPEG encoding failed')
    body = {'model': model, 'temperature': 0.0, 'max_tokens': 1024, 'messages': [{'role': 'user', 'content': [
        {'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode()}},
        {'type': 'text', 'text': prompt}]}]}
    response = requests.post(f'{endpoint}/v1/chat/completions', json=body, timeout=timeout)
    response.raise_for_status()
    return response.json()['choices'][0]['message']['content']


def to_1000(box, width, height, pixels):
    # Qwen2.5-VL answers in pixels of its resized input, which is within a few px of the frame's size.
    return [box[0] * 1000 // width, box[1] * 1000 // height, box[2] * 1000 // width, box[3] * 1000 // height] \
        if pixels else box


def summary(label):
    seen = [b['name'] for b in label['bottles'] if b['visible']]
    return (f"bottles={','.join(seen) or '-'} glasses={len(label['glasses'])} "
            f"gripper={label['in_gripper']['value']} obstruction={label['obstruction']['value']}")


def draw(frame, label, pixels):
    h, w = frame.shape[:2]
    for item in label['bottles'] + label['glasses']:
        if item['bbox']:
            x0, y0, x1, y1 = to_1000(item['bbox'], w, h, pixels)
            colour = (255, 255, 0) if item['name'] == GLASS else (0, 0, 255)
            p0, p1 = (x0 * w // 1000, y0 * h // 1000), (x1 * w // 1000, y1 * h // 1000)
            cv2.rectangle(frame, p0, p1, colour, 2)
            cv2.putText(frame, f"{item['name']} {item['confidence']:.2f}", (p0[0], p0[1] + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--source', required=True)
    parser.add_argument('--endpoint', default='http://localhost:8100')
    parser.add_argument('--model', default='scene-vlm')
    parser.add_argument('--interval', type=float, default=0.0, help='minimum seconds between requests')
    parser.add_argument('--bbox-pixels', action='store_true', help='model answers boxes in pixels (Qwen2.5-VL)')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--show', action='store_true', help='window with the last answer drawn on the frame')
    parser.add_argument('--log', help='append every answer as a JSON line')
    opts = parser.parse_args()

    camera = LatestFrame(opts.source)
    log = open(opts.log, 'a') if opts.log else None
    latencies = []
    while True:
        frame = camera.get()
        if frame is None:
            time.sleep(0.1)
            continue
        start = time.time()
        try:
            answer = ask(opts.endpoint, opts.model, frame, REAL_BAR_PROMPT, opts.timeout)
        except (requests.RequestException, ValueError) as error:
            print(f'request failed: {error}')
            time.sleep(1.0)
            continue
        latency = time.time() - start
        latencies.append(latency)
        label = canonical_label(answer)
        p50 = sorted(latencies)[len(latencies) // 2]
        print(f"{time.strftime('%H:%M:%S')} {latency:5.2f}s (p50 {p50:.2f}s, n={len(latencies)}) "
              + (summary(label) if label else 'INVALID: ' + answer[:120].replace('\n', ' ')))
        if log:
            log.write(json.dumps({'t': start, 'latency': latency, 'answer': answer, 'label': label}) + '\n')
            log.flush()
        if opts.show:
            cv2.imshow('scene-vlm', draw(frame, label, opts.bbox_pixels) if label else frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        time.sleep(max(0.0, opts.interval - latency))


if __name__ == '__main__':
    main()
