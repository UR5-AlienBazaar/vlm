from dataclasses import dataclass


@dataclass(frozen=True)
class Detection:
    box: tuple[int, int, int, int]
    confidence: float
    track_id: int | None = None

    def contains(self, x: int, y: int) -> bool:
        x0, y0, x1, y1 = self.box
        return x0 <= x <= x1 and y0 <= y <= y1
