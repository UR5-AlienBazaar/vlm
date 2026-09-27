#!/usr/bin/env python3
"""Deduplicate quality-gated collected crops and split whole sessions."""
import argparse
from pathlib import Path

from bottle_vision.dataset import materialize_splits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path, help="data/bottles/metadata.jsonl from live_bottle_label.py")
    parser.add_argument("--out", type=Path, default=Path("data/bottles/split"))
    parser.add_argument("--val", type=float, default=.15)
    parser.add_argument("--test", type=float, default=.15)
    args = parser.parse_args()
    print(materialize_splits(args.manifest, args.out, args.val, args.test))


if __name__ == "__main__":
    main()
