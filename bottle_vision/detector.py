"""Pretrained YOLO bottle detection. A custom detector can implement `detect`."""
from .types import Detection


class YOLOBottleDetector:
    def __init__(self, weights: str = "yolo11n.pt", confidence: float = 0.35):
        from ultralytics import YOLO
        self.model, self.confidence = YOLO(weights), confidence

    def detect(self, frame) -> list[Detection]:
        result = self.model(frame, conf=self.confidence, verbose=False)[0]
        names = result.names
        found = []
        for box in result.boxes:
            name = str(names[int(box.cls[0])]).lower()
            if name != "bottle":
                continue
            x0, y0, x1, y1 = (int(v) for v in box.xyxy[0].tolist())
            found.append(Detection((x0, y0, x1, y1), float(box.conf[0])))
        return found
