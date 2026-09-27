import pytest

from pour_gate import check, taught_point

TAUGHT = (900.0, 300.0)
NOW = 1000.0


def reading(*xys, at=NOW):
    return {"circles": [{"x_mm": x, "y_mm": y} for x, y in xys], "updated_at": at}


def test_glass_on_the_spot_passes():
    assert check([reading((910, 305))] * 5, TAUGHT, NOW) is None


def test_glass_moved_is_refused():
    assert "25 mm" in check([reading((925, 300))] * 5, TAUGHT, NOW)


def test_nearest_circle_counts_when_others_are_in_view():
    assert check([reading((700, 100), (905, 300))] * 5, TAUGHT, NOW) is None


def test_flickering_false_circle_does_not_pass():
    flicker = [reading((902, 301)), reading((1040, 250)), reading((902, 301))]
    assert "mm from the taught" in check(flicker, TAUGHT, NOW)


def test_no_glass_is_refused():
    assert "no glass" in check([reading()], TAUGHT, NOW)


def test_stale_or_future_reading_is_refused():
    assert "old" in check([reading((900, 300), at=NOW - 5)], TAUGHT, NOW)
    assert "old" in check([reading((900, 300), at=NOW + 5)], TAUGHT, NOW)
    assert "old" in check([{"circles": [{"x_mm": 900, "y_mm": 300}]}], TAUGHT, NOW)


def test_teach_averages_the_hinted_circle_and_ignores_the_rest():
    bodies = [reading((948, 512), (840, 350)), reading((952, 508), (1060, 270))]
    assert taught_point(bodies, (940, 500)) == (950.0, 510.0)


def test_teach_refuses_a_wandering_or_missing_circle():
    with pytest.raises(ValueError, match="moves"):
        taught_point([reading((940, 500)), reading((970, 500))], (955, 500))
    with pytest.raises(ValueError, match="every reading"):
        taught_point([reading((950, 510)), reading((700, 100))], (950, 510))
