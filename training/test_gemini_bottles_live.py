from training.gemini_bottles_live import to_objects


def test_to_objects_scales_and_filters():
    answer = '[{"label": "liqueur", "box_2d": [100, 500, 300, 600]}, {"label": "cup", "box_2d": [0, 0, 10, 10]}, {"label": "cola"}]\n[]'
    [row] = to_objects(answer, 1280, 720)
    assert row["label"] == "liqueur"
    assert row["bbox"] == [640, 72, 768, 216]
    assert (row["x"], row["y"]) == (704, 144)
