"""Read the text on a bottle label locally with PaddleOCR; no network once the models are cached.

Off unless paddleocr is installed (pip install paddlepaddle paddleocr).
"""
import os

import cv2

# ponytail: calibration knob; labels are small in a station crop, and 2x read
# the brand and type on photos/ where 1x missed them. 3x reads a little more
# at twice the time (~3 s a crop on CPU).
SCALE = 2.0


def make_reader():
    """Return read(bgr) -> (label text, None) | (None, why), or None when not installed."""
    try:
        from paddleocr import PaddleOCR
    except ImportError:
        return None
    # Skips PaddleX's check of its model host on every start, so an offline start doesn't wait.
    os.environ.setdefault('PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK', 'True')
    ocr = PaddleOCR(use_doc_orientation_classify=False, use_doc_unwarping=False,
                    use_textline_orientation=False,
                    # Paddle 3.x's oneDNN kernels crash on the detector (NotImplementedError).
                    enable_mkldnn=False)

    def read(bgr):
        big = cv2.resize(bgr, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_CUBIC)
        try:
            lines = ocr.predict(big)[0]['rec_texts']
        except (RuntimeError, NotImplementedError, ValueError) as exc:
            return None, f'ocr: {exc}'
        return ' '.join(lines), None

    return read
