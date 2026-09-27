from safe_make import bottles_for, refusals


def row(label, best_guess, score, track_id=1, found=True):
    return {"track_id": track_id, "label": label, "best_guess": best_guess, "score": score, "found": found}


MENU = {"drinks": [{"drink": "vodka_sprite", "scripts": ["grab_vodka", "pour_vodka", "return_vodka",
                                                           "grab_sprite", "pour_sprite", "return_sprite"]}]}


def test_bottles_come_from_grab_scripts():
    assert bottles_for("vodka_sprite", MENU) == ["vodka", "sprite"]


def test_go_when_every_bottle_is_confidently_seen():
    assert refusals(["vodka", "sprite"], [row("vodka", "vodka", .9), row("7up", "7up", .8, 2)], .6) == []


def test_missing_bottle_refuses():
    assert refusals(["vodka", "sprite"], [row("vodka", "vodka", .9)], .6) == ["no sprite bottle seen on the bar"]


def test_unsure_classifier_refuses():
    [problem] = refusals(["vodka"], [row(None, "vodka", .45, 3)], .6)
    assert problem.startswith("not sure bottle #3 is the vodka")


def test_client_threshold_overrides_a_looser_server():
    assert refusals(["vodka"], [row("vodka", "vodka", .5)], .6)


def test_lost_track_is_not_a_sighting():
    assert refusals(["vodka"], [row("vodka", "vodka", .9, found=False)], .6) == ["no vodka bottle seen on the bar"]
