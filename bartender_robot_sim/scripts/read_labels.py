#!/usr/bin/env python3
"""Read bottle labels in a photo the way the server reads a station crop.

    read_labels.py PHOTO [--box u0,u1,v0,v1 ...] [--bottles bottles.yaml]

Each --box is one bottle, in pixels of the photo scaled to at most --max-side
(the stand camera's resolution is what matters, not the phone's). Without
--box the whole photo is one crop. OCR runs when paddleocr is installed, and
Gemini is the fallback when GEMINI_API_KEY and GEMINI_VISION_MODEL are set
(--no-gemini to see OCR alone). Reads any format Pillow opens, including
phone .dng.
"""
import argparse
import dataclasses
import json
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, 'ros2_ws', 'src', 'bartender_api'))

from bartender_api.drink import jev, label, ocr, vlm            # noqa: E402

DEFAULT_BOTTLES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               os.pardir, 'ros2_ws', 'src', 'bartender_api',
                               'config', 'bottles.yaml')


def load_bgr(path, max_side):
    image = Image.open(path).convert('RGB')
    image.thumbnail((max_side, max_side))
    return np.ascontiguousarray(np.asarray(image)[..., ::-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('photo')
    parser.add_argument('--box', action='append', default=[],
                        help='u0,u1,v0,v1 around one bottle (repeatable)')
    parser.add_argument('--bottles', default=DEFAULT_BOTTLES)
    parser.add_argument('--max-side', type=int, default=1600)
    parser.add_argument('--no-gemini', action='store_true')
    parser.add_argument('--no-jev', action='store_true',
                        help='no Jev fallback even if JEV_KEY is set')
    opts = parser.parse_args()

    inventory = label.load_inventory(opts.bottles)
    read_text = ocr.make_reader()
    ask = None if opts.no_gemini else vlm.make_asker(label.PROMPT,
                                                     label.schema(inventory))
    deciders = (label.fuzzy, *filter(None, [None if opts.no_jev else jev.make_jev()]))
    if read_text is None and ask is None:
        sys.exit('install paddleocr, or set GEMINI_API_KEY and GEMINI_VISION_MODEL')
    bgr = load_bgr(opts.photo, opts.max_side)
    h, w = bgr.shape[:2]
    boxes = [tuple(map(int, b.split(','))) for b in opts.box] or [(0, w, 0, h)]
    for u0, u1, v0, v1 in boxes:
        got = label.read_label(bgr[v0:v1, u0:u1], inventory, read_text, ask, deciders)
        print(json.dumps({'box': [u0, u1, v0, v1], **dataclasses.asdict(got)},
                         ensure_ascii=False))


if __name__ == '__main__':
    main()
