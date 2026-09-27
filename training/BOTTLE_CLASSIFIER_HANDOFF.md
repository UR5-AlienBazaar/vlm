# Handoff: bottle/drink classifier

Not started yet. This is the plan, agreed in session, for whoever picks it
up next.

## Goal

Given a crop of one bottle from the overhead camera (`cam0_upside`), say
which drink it is. No localization needed -- unlike the cup tracker, we
don't need the bottle's pixel centre, just "which of these labels is this."

The six served drinks (see `training/vlm_labels.py`'s `LABEL_IDS` comment,
which is the canonical list -- never renumber it):

| label     | real bottle              |
|-----------|---------------------------|
| `whiskey` | Jack Daniel's              |
| `beer`    | Heineken                   |
| `vodka`   | Zubrowka                   |
| `liqueur` | Jägermeister                |
| `gin`     | Tenjaku gin                 |
| `wine`    | Frontera white wine         |

`cola` is retired (kept only so old captures still decode). Anything else on
the table -- Ballantine's, a Mirinda bottle, etc. -- is a **distractor**:
the model must say "not one of these six," not force a nearest-guess label.

## Recommended approach

Same building block as `training/cup_center_live.py` (DINOv2, frozen,
already proven to work well on this exact camera/lighting in this repo),
adapted from *localization* to *classification*:

1. For each class, embed a handful of reference crops with DINOv2
   (`facebook/dinov2-small`, whole-crop embedding via `pooler_output`, not
   the per-patch grid `cup_center_live.py` uses) and average into one
   reference vector per class.
2. For a new crop, embed it the same way, cosine-similarity it against
   every class's reference vector, and take the best match -- but only
   if it clears a margin over the *second*-best match and an absolute
   floor. Below that, report "distractor / unknown," not a forced guess.
   (This mirrors the cup tracker's `found: false` discipline -- see
   `training/CUP_TRACKER.md`'s "Consuming the cup position" section for why
   that mattered in practice.)
3. Reference crops already exist for four of six classes:
   `photos/bottles/{vodka,liqueur,gin,jack_daniels}*.jpg`. **Missing:**
   `beer` (Heineken) and `wine` (Frontera) have no reference photos yet --
   take some before this can classify those two. Ballantine's distractor
   crops exist (`ballantines_distractor_*.jpg`); a Mirinda distractor
   crop does not yet -- worth adding since it's now on the table, as one
   more "known distractor" the model should confidently reject rather than
   misclassify as `cola` or anything else.

## Why not other approaches

- **A full VLM call per frame** (reusing `training/prelabel_real.py`'s
  `REAL_BAR_PROMPT`) works and already speaks these exact label names, but
  is much heavier per-frame than an embedding lookup. Fine for offline
  pre-labelling (that script's actual job); too slow for a live loop.
- **Zero-shot detectors** (YOLO/Grounding DINO/OWLv2) were tried for the cup
  and were unreliable on this cluttered table -- see `CUP_TRACKER.md`'s
  history. No reason to expect better luck on bottles, which are more
  visually similar to each other than the cup was to its confusers.

## Where crops come from

Either:
- Hand-crop a box out of a `cam0_upside` frame per bottle (same pattern as
  `cup_center_live.py --ref/--ref-box`), or
- Reuse `photos/bottles/*.jpg`, which are already single-bottle crops --
  simpler to start from since no boxing is needed.

## Suggested shape

A new `training/bottle_classifier.py`, structured like
`cup_center_live.py`: a `PatchMatcher`-equivalent wrapper around DINOv2, an
`enrol()` that builds per-class reference vectors from
`--ref-dir photos/bottles/` (grouped by filename prefix) plus any new
crops, and a `classify(crop) -> (label_or_None, best_score, margin)`
function with a small `demo()`/test asserting a known crop classifies
correctly and a clearly-different one doesn't.

## Open questions for whoever builds this

- Live loop (poll a camera crop continuously, like the cup tracker) or
  on-demand (classify one crop, return, done)? Session didn't settle this
  because it depends on how the pour planner wants to call it.
- Where do "which bottle is at which position" come from, if the planner
  needs to know both identity and rough location eventually? Out of scope
  for this handoff (explicitly no localization needed per this task), but
  likely the next ask once classification works.
