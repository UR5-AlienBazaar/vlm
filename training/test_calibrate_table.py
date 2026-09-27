import numpy as np
import pytest

from bottle_vision import table

# A made-up camera: 0.8 mm/px, rotated 90 degrees, offset, with mild perspective.
TRUE_H = np.array([[0.0, -0.8, 600.0], [0.8, 0.0, -400.0], [1e-5, 2e-5, 1.0]])
PIXELS = np.array([[100, 80], [1180, 90], [1200, 650], [90, 640], [640, 360], [400, 500]], float)


def test_fit_recovers_mapping_and_reports_small_error():
    base = table.apply(TRUE_H, PIXELS)
    H, errors = table.fit(PIXELS, base)
    assert errors.max() < 1e-3
    assert table.apply(H, (700, 200)) == pytest.approx(table.apply(TRUE_H, (700, 200)), abs=1e-3)


def test_typo_shows_up_as_large_error():
    base = table.apply(TRUE_H, PIXELS); base[2, 0] += 100
    _, errors = table.fit(PIXELS, base)
    assert errors.max() > 20


def test_needs_four_points():
    with pytest.raises(ValueError):
        table.fit(PIXELS[:3], table.apply(TRUE_H, PIXELS[:3]))
