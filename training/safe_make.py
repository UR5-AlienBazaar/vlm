#!/usr/bin/env python3
"""Order a drink from bartender_api, but only if its bottles are on camera.

    python training/safe_make.py whiskey_cola

Reads the drink's grab_<bottle> scripts from GET /drinks, then checks the
live classifier's GET /objects (live_bottle_infer.py) for a confidently
labelled bottle of each. If any is missing or unsure, it refuses without
calling POST /make, so the arm never reaches for a bottle that is not there.
"""
import argparse
import json
import sys
import urllib.error
import urllib.request

# The menu names the sim's mixers; the classifier learned the real bar's bottles.
CLASSIFIER_LABEL = {"sprite": "7up", "fanta": "mirinda"}


def call(url, body=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except urllib.error.HTTPError as err:  # bartender_api's 409s still carry JSON
        return json.load(err)


def bottles_for(drink, menu):
    for entry in menu["drinks"]:
        if entry["drink"] == drink:
            return [script.removeprefix("grab_") for script in entry["scripts"] if script.startswith("grab_")]
    raise KeyError(drink)


def refusals(bottles, objects, min_score):
    """One message per bottle that is not seen with a label at or above min_score; empty means go."""
    # Held rows (found=False) are the tracker's memory of a lost box, not a sighting.
    seen = [row for row in objects if row.get("found")]
    problems = []
    for bottle in bottles:
        label = CLASSIFIER_LABEL.get(bottle, bottle)
        if any(row["label"] == label and row["score"] >= min_score for row in seen):
            continue
        maybe = [row for row in seen if row["best_guess"] == label]
        if maybe:
            best = max(maybe, key=lambda row: row["score"])
            problems.append(f"not sure bottle #{best['track_id']} is the {bottle} "
                            f"(classifier {best['score']:.0%}, need {min_score:.0%})")
        else:
            problems.append(f"no {bottle} bottle seen on the bar")
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("drink")
    parser.add_argument("--api", default="http://127.0.0.1:8090")
    parser.add_argument("--objects", default="http://127.0.0.1:8770/objects")
    # Checked here as well as in the server's --threshold so a looser server cannot let a guess through.
    parser.add_argument("--min-score", type=float, default=.6)
    args = parser.parse_args()

    try:
        bottles = bottles_for(args.drink, call(args.api + "/drinks"))
        objects = call(args.objects)["objects"]
    except KeyError:
        sys.exit(f"refused: {args.drink!r} is not on the menu")
    except (urllib.error.URLError, TimeoutError, ValueError) as exc:
        sys.exit(f"refused: cannot check the bar ({exc}); nothing moved")

    problems = refusals(bottles, objects, args.min_score)
    if problems:
        sys.exit(f"refused {args.drink}: " + "; ".join(problems) + ". Nothing moved.")

    result = call(args.api + "/make", {"drink": args.drink}, timeout=900)
    print(result["message"])
    sys.exit(0 if result["ok"] else 1)


if __name__ == "__main__":
    main()
