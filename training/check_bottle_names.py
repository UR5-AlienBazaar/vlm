#!/usr/bin/env python3
"""Check whether the served VLM names the bottles on the table correctly.

Scores which drinks the VLM says are present in each human-reviewed frame
against annotations.jsonl, then (--live) runs the same prompt on the robot's
camera stream so you can watch it against the real table.

    python training/check_bottle_names.py data/overhead_raw/20260927 --first 127
    python training/check_bottle_names.py --live http://10.42.0.200:8765/cam0_upside
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from training.eval_bottles import PROMPT, STRICT_CONTRACT, infer

NAMES = {"whiskey", "cola", "vodka", "liqueur", "beer", "gin", "wine", "mirinda", "7up"}


def predicted_names(answer: str) -> set[str]:
    match = re.search(r"\{.*\}", answer, re.S)  # tolerate ```json fences
    try:
        bottles = json.loads(match.group(0)).get("bottles", []) if match else []
    except (json.JSONDecodeError, AttributeError):
        return set()
    return {b.get("name") for b in bottles if isinstance(b, dict) and b.get("visible", True)} & NAMES


def reviewed_frames(root: Path, first: int) -> list[tuple[str, set[str]]]:
    last = {}
    for line in (root / "annotations.jsonl").read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        if record.get("reviewed"):
            last[record["image"]] = {a["label"] for a in record["annotations"]} & NAMES
    order = [p.relative_to(root).as_posix() for p in sorted(root.glob("*/*.jpg"))]
    return [(key, last[key]) for key in order if key in last][:first]


def score(rows: list[tuple[set[str], set[str]]]) -> dict:
    tp, fp, fn = Counter(), Counter(), Counter()
    for truth, guess in rows:
        tp.update(truth & guess); fp.update(guess - truth); fn.update(truth - guess)
    return {"exact_frames": sum(t == g for t, g in rows), "frames": len(rows),
            "false_go": sum(fp.values()), "tp": tp, "fp": fp, "fn": fn}


def report(result: dict) -> None:
    print(f"\nexact frames {result['exact_frames']}/{result['frames']}   "
          f"false 'go' (named but absent) {result['false_go']}")
    print(f"{'drink':<9}{'truth':>6}{'hit':>5}{'miss':>6}{'false':>7}")
    for name in sorted(NAMES, key=lambda n: -(result["tp"][n] + result["fn"][n])):
        truth = result["tp"][name] + result["fn"][name]
        print(f"{name:<9}{truth:>6}{result['tp'][name]:>5}{result['fn'][name]:>6}{result['fp'][name]:>7}")


def run_offline(args, prompt: str) -> None:
    rows, out = [], args.out.open("w", encoding="utf-8")
    for index, (key, truth) in enumerate(reviewed_frames(args.root, args.first), 1):
        answer = infer(args.endpoint, args.model, args.root / key, args.max_pixels, prompt)
        guess = predicted_names(answer); rows.append((truth, guess))
        out.write(json.dumps({"image": key, "truth": sorted(truth), "guess": sorted(guess), "answer": answer}) + "\n")
        flag = "ok " if truth == guess else "BAD"
        print(f"[{index}] {flag} {key}  missing={sorted(truth - guess)} extra={sorted(guess - truth)}", flush=True)
    report(score(rows))


def run_live(args, prompt: str) -> None:
    import cv2
    from bottle_vision.camera import frames
    path = Path(tempfile.gettempdir()) / "check_bottle_names_live.jpg"
    for frame in frames(args.live):
        cv2.imwrite(str(path), frame)
        started = time.time()
        guess = predicted_names(infer(args.endpoint, args.model, path, args.max_pixels, prompt))
        print(f"{time.strftime('%H:%M:%S')} ({time.time() - started:.1f}s) {sorted(guess) or 'nothing'}", flush=True)
        if args.save_dir:
            args.save_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.save_dir / f"{time.strftime('%H%M%S')}.jpg"), frame)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("root", type=Path, nargs="?", help="folder with annotations.jsonl and <camera>/*.jpg")
    parser.add_argument("--first", type=int, default=127, help="only the first N reviewed frames (the corrected ones)")
    parser.add_argument("--live", help="MJPEG URL; skip scoring and print names for the live stream")
    parser.add_argument("--save-dir", type=Path, help="with --live, keep each frame so it can be labelled later")
    parser.add_argument("--endpoint", default="http://127.0.0.1:8092", help="ssh -L 8092:localhost:8092 training-center-point")
    parser.add_argument("--model", default="qwen3-vl-4b")
    parser.add_argument("--max-pixels", type=int, default=3_000_000)
    parser.add_argument("--out", type=Path, default=Path("check_bottle_names.jsonl"))
    args = parser.parse_args()
    prompt = PROMPT + STRICT_CONTRACT
    if args.live:
        run_live(args, prompt)
    elif args.root:
        run_offline(args, prompt)
    else:
        parser.error("give a labelled folder or --live URL")


if __name__ == "__main__":
    assert predicted_names('```json\n{"bottles":[{"name":"gin","visible":true},{"name":"ballantines"}]}\n```') == {"gin"}
    assert predicted_names("no json") == set()
    assert score([({"gin"}, {"gin", "cola"})])["false_go"] == 1
    main()
