"""ByteTrack adapter; tracking is deliberately independent of label state."""
from .types import Detection


class YOLOByteTracker:
    """Use Ultralytics' maintained ByteTrack implementation with persistent IDs."""
    def __init__(self, weights: str = "yolo11n.pt", confidence: float = 0.35,
                 tracker: str = "bytetrack.yaml", image_size: int = 640):
        from ultralytics import YOLO
        self.model, self.confidence, self.tracker, self.image_size = YOLO(weights), confidence, tracker, image_size
        # COCO weights number bottle 39; the fine-tuned detector has only class 0.
        self.classes = [index for index, name in self.model.names.items() if name == "bottle"]

    def update(self, frame) -> list[Detection]:
        result = self.model.track(frame, persist=True, tracker=self.tracker,
                                  classes=self.classes, conf=self.confidence, imgsz=self.image_size, verbose=False)[0]
        if result.boxes is None or result.boxes.id is None:
            return []
        return [Detection(tuple(int(v) for v in box.xyxy[0].tolist()), float(box.conf[0]), int(box.id[0]))
                for box in result.boxes]
