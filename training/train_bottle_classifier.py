#!/usr/bin/env python3
"""Train a DINOv2 bottle head, optionally unfreezing its final four blocks."""
import argparse
import json
from pathlib import Path

import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from tqdm import tqdm

from bottle_vision.classifier import DinoBottleClassifier


def transform(train: bool, size: int):
    if train:
        return transforms.Compose([transforms.RandomResizedCrop(size, scale=(.75, 1.0)), transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(8), transforms.ColorJitter(.18, .18, .12), transforms.GaussianBlur(3, (.1, 1.0)),
            transforms.ToTensor(), transforms.Normalize((.485, .456, .406), (.229, .224, .225))])
    return transforms.Compose([transforms.Resize(int(size * 1.14)), transforms.CenterCrop(size), transforms.ToTensor(),
                               transforms.Normalize((.485, .456, .406), (.229, .224, .225))])


def score(model, loader, device):
    model.eval(); correct = total = 0
    with torch.inference_mode():
        for images, labels in loader:
            correct += (model(images.to(device)).argmax(1).cpu() == labels).sum().item(); total += len(labels)
    return correct / total if total else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/bottles/split"))
    parser.add_argument("--out", type=Path, default=Path("checkpoints/bottles"))
    parser.add_argument("--model", default="facebook/dinov2-large")
    parser.add_argument("--stage", choices=("head", "finetune"), default="head")
    parser.add_argument("--epochs", type=int, default=12); parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--image-size", type=int, default=518); parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--head-lr", type=float, default=3e-4); parser.add_argument("--backbone-lr", type=float, default=1e-5)
    parser.add_argument("--weight-decay", type=float, default=.01)
    parser.add_argument("--allow-no-validation", action="store_true",
                        help="write a development-only checkpoint when no session-held-out validation set exists")
    args = parser.parse_args()
    train_set = datasets.ImageFolder(args.data / "train", transform(True, args.image_size))
    has_val_images = any(path.is_file() for path in (args.data / "val").glob("*/*"))
    val_set = datasets.ImageFolder(args.data / "val", transform(False, args.image_size)) if has_val_images else None
    has_validation = val_set is not None and bool(val_set) and train_set.classes == val_set.classes
    if not train_set or (not has_validation and not args.allow_no_validation):
        raise SystemExit("need non-empty train/val folders with every class; collect separate sessions first")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = DinoBottleClassifier(args.model, train_set.classes).to(device); model.set_stage(args.stage == "finetune")
    groups = [{"params": model.head.parameters(), "lr": args.head_lr}]
    if args.stage == "finetune": groups.append({"params": [p for p in model.backbone.parameters() if p.requires_grad], "lr": args.backbone_lr})
    optimizer = torch.optim.AdamW(groups, weight_decay=args.weight_decay); criterion = nn.CrossEntropyLoss()
    train_loader = DataLoader(train_set, args.batch_size, shuffle=True, num_workers=args.workers, pin_memory=device == "cuda")
    val_loader = DataLoader(val_set, args.batch_size, num_workers=args.workers, pin_memory=device == "cuda") if has_validation else None
    args.out.mkdir(parents=True, exist_ok=True); history = []; best = None
    for epoch in range(args.epochs):
        model.train(); losses = []
        for images, labels in tqdm(train_loader, desc=f"epoch {epoch + 1}/{args.epochs}"):
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device, dtype=torch.bfloat16, enabled=device == "cuda"):
                loss = criterion(model(images.to(device)), labels.to(device))
            loss.backward(); optimizer.step(); losses.append(float(loss.detach()))
        accuracy = score(model, val_loader, device) if val_loader else None
        row = {"epoch": epoch + 1, "loss": sum(losses) / len(losses), "val_accuracy": accuracy}; history.append(row); print(row)
        state = {"model": model.state_dict(), "classes": train_set.classes, "model_id": args.model, "image_size": args.image_size, "stage": args.stage, "validated": has_validation}
        torch.save(state, args.out / "last.pt")
        if best is None or (accuracy is not None and accuracy > best):
            best = accuracy; torch.save(state, args.out / "best.pt")
    (args.out / "history.json").write_text(json.dumps(history, indent=2))


if __name__ == "__main__": main()
