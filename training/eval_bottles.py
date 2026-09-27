#!/usr/bin/env python3
"""Run a vLLM-served bartender model over a folder of real bottle photos.

Example (on the GPU instance):
    python eval_bottles.py /home/shadeform/bottles --out bottles_sft_r1.jsonl
"""
import argparse
import base64
from io import BytesIO
import json
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

PROMPT = (
    "Describe the bar scene as JSON with keys bottles (name, visible, confidence, bbox), "
    "glasses (name, visible, confidence, bbox), in_gripper ({value: bottle name or null, "
    "confidence}) and obstruction ({value: true if something blocks a bottle or a glass, "
    "confidence}). bbox is [x0, y0, x1, y1] in 0-1000 image coordinates. confidence is 0-1: "
    "how sure you are of visible or value."
)
STRICT_CONTRACT = (
    "\n\nUse this robot inventory contract exactly. A bottle entry's name must be one of: "
    "whiskey, cola, vodka, liqueur, beer, gin, wine, mirinda, 7up. Whiskey is reserved only for a Jack Daniel's "
    "bottle: square with the black-and-white Jack Daniel's label. Never use whiskey as a generic "
    "category for another whisky. Map Jack Daniel's to whiskey; Zubrowka to vodka; "
    "Jagermeister (including Jägermeister) to liqueur; Heineken to beer; Tenjaku to gin; and "
    "Frontera white wine to wine, Mirinda to mirinda, and 7UP to 7up. Ballantine's (brown bottle, cream label and white cap) is a "
    "distractor: never report it as whiskey or as any bottle. Do not report any unlisted brand "
    "or object. Do not duplicate a bottle name. "
    "A human hand is not the robot gripper, so in_gripper must be null unless a robot gripper is "
    "visibly holding a target bottle. Return JSON only."
)
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def image_payload(image: Path, max_pixels: int) -> tuple[str, str]:
    """Return a JPEG data payload small enough for the server's visual-token budget."""
    from PIL import Image, ImageOps

    with Image.open(image) as source:
        rgb = ImageOps.exif_transpose(source).convert("RGB")
        pixels = rgb.width * rgb.height
        if pixels > max_pixels:
            scale = (max_pixels / pixels) ** 0.5
            size = (round(rgb.width * scale), round(rgb.height * scale))
            rgb = rgb.resize(size, Image.Resampling.LANCZOS)
        data = BytesIO()
        rgb.save(data, format="JPEG", quality=92)
    return "image/jpeg", base64.b64encode(data.getvalue()).decode("ascii")


def infer(endpoint: str, model: str, image: Path, max_pixels: int, prompt: str) -> str:
    content_type, encoded = image_payload(image, max_pixels)
    request_body = {
        "model": model,
        "temperature": 0.0,
        "max_tokens": 1024,
        "messages": [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": f"data:{content_type};base64,{encoded}"}},
            {"type": "text", "text": prompt},
        ]}],
    }
    request = Request(
        endpoint.rstrip("/") + "/v1/chat/completions",
        data=json.dumps(request_body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=300) as response:
            payload = json.load(response)
    except HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"vLLM HTTP {error.code}: {detail}") from error
    return payload["choices"][0]["message"]["content"]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("photos", type=Path)
    parser.add_argument("--endpoint", default="http://127.0.0.1:8101")
    parser.add_argument("--model", default="sft-r1")
    parser.add_argument("--out", type=Path, default=Path("bottles_sft_r1.jsonl"))
    parser.add_argument("--limit", type=int, help="Run only the first N photos (for diagnostics).")
    parser.add_argument("--max-pixels", type=int, default=3_000_000,
                        help="Maximum image pixels sent to vLLM (default: 3,000,000).")
    parser.add_argument("--strict-contract", action="store_true",
                        help="Constrain bottle names and known real-bar mappings in the request.")
    parser.add_argument("--prompt-file", type=Path, help="Use this prompt instead of the built-in one.")
    args = parser.parse_args()

    photos = sorted(p for p in args.photos.rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES)
    if not photos:
        raise SystemExit(f"No JPG, JPEG, or PNG images under {args.photos}")
    if args.limit is not None:
        photos = photos[:args.limit]
    prompt = PROMPT + STRICT_CONTRACT if args.strict_contract else PROMPT
    if args.prompt_file:
        prompt = args.prompt_file.read_text(encoding="utf-8")

    with args.out.open("w", encoding="utf-8") as out:
        for index, photo in enumerate(photos, start=1):
            try:
                answer = infer(args.endpoint, args.model, photo, args.max_pixels, prompt)
                record = {"image": photo.name, "prompt_mode": "strict" if args.strict_contract else "default",
                          "answer": answer}
                try:
                    record["parsed"] = json.loads(answer)
                except json.JSONDecodeError:
                    record["parse_error"] = True
                print(f"[{index}/{len(photos)}] {photo.name}: {'valid JSON' if 'parsed' in record else 'invalid JSON'}", flush=True)
            except Exception as error:  # Preserve individual failures for review.
                record = {"image": photo.name, "error": repr(error)}
                print(f"[{index}/{len(photos)}] {photo.name}: ERROR {error}", flush=True)
            out.write(json.dumps(record) + "\n")
            out.flush()


if __name__ == "__main__":
    main()
