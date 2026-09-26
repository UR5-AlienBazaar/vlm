"""Ask a Gemini vision model about a camera crop; label.py uses it to read bottle labels.

Off unless GEMINI_API_KEY and GEMINI_VISION_MODEL are both set. Every call
sends the crop to Google.
"""
import json
import os

import cv2

# ponytail: calibration knob; a label read waits this long for Gemini at most.
TIMEOUT_S = 15.0


def make_asker(prompt, schema):
    """Return ask(bgr) -> (answer dict, None) | (None, why), or None when not configured."""
    model = os.environ.get('GEMINI_VISION_MODEL')
    if not model or not os.environ.get('GEMINI_API_KEY'):
        return None
    import httpx
    from google import genai
    from google.genai import errors, types
    client = genai.Client(http_options=types.HttpOptions(timeout=int(TIMEOUT_S * 1000)))
    config = types.GenerateContentConfig(
        response_mime_type='application/json', response_schema=schema)

    def ask(bgr):
        ok, jpg = cv2.imencode('.jpg', bgr)
        if not ok:
            return None, 'could not encode the crop'
        image = types.Part.from_bytes(data=jpg.tobytes(), mime_type='image/jpeg')
        try:
            response = client.models.generate_content(
                model=model, contents=[image, prompt], config=config)
            # text is None when the reply was blocked or empty.
            return json.loads(response.text or ''), None
        except (errors.APIError, httpx.HTTPError, json.JSONDecodeError) as exc:
            return None, f'gemini: {exc}'

    return ask
