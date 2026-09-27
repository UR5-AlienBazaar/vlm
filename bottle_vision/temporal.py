from collections import deque
import numpy as np


class ProbabilitySmoother:
    def __init__(self, history: int = 15):
        self.history, self.values = history, {}

    def update(self, track_id: int, probabilities: np.ndarray) -> np.ndarray:
        history = self.values.setdefault(track_id, deque(maxlen=self.history))
        history.append(np.asarray(probabilities, dtype=float))
        return np.mean(history, axis=0)

    def retain(self, active_ids: set[int]) -> None:
        self.values = {key: value for key, value in self.values.items() if key in active_ids}
