# Control API — proposal

**Status: Phase B is built, and Phase C has started -- simple movement
only.** `GET /world`, `GET /state`, `POST /can` and the error-taxonomy
classifier (Phase B) all exist and are tested against the real running
sim. `POST /move/point`, `POST /move/jog` and `POST /gripper` (part of
Phase C) now exist too, and this process moves the robot through them --
see "Safety". They are a thin wrapper over `Pendant.dispatch()`
(`bartender_api/movement.py`), scoped to exactly what Pendant already does
safely: drive to a taught point, jog relative to where the arm is, or move
the gripper. Arbitrary joint or Cartesian targets (`/move/joints`,
`/move/tool` in the sketch below), `/do` with jobs, pipelines, a global
stop and an action budget are still design only.

Three things worth knowing before extending it, found while building
Phase B rather than guessed in advance:

- **`layout.servicing_arms()` answers a narrower question than
  "reachable".** It is APPROACH_WINDOW: can this be side-grasped off the
  bottle line with the fixed tool orientation that grasp uses. Applied to
  the glass or the opener holster -- neither of which is side-grasped, one
  tilted into, one descended onto -- it says no arm can reach either,
  which is false; the real pour and open skills reach them every day.
  `bartender_api/reach.py` is a second, plain radial-reach check for
  exactly those two, and both `/world`'s `reachable_by` and `/can` use it.
  A generic "/world always uses servicing_arms" implementation ships this
  bug; it was caught by a live curl against the sim before it caught
  anyone else.
- **`Arm.last_error` and `OpenActionServer._why`/`_why_any`** (in
  `bartender_open`) now thread a sub-call's specific logged reason into
  the coarse "could not pick up the opener" style messages the action
  result carries. Before this, GRIPPER_NOT_FOLLOWING and
  GRASP_STOPPED_WIDE -- split apart specifically because conflating them
  sent one investigation to the wrong half of the robot -- collapsed back
  into the same one sentence the moment they reached an action result,
  which is what the classifier below actually reads.
- **`pour_action_server` now carries the same `last_error`/`_why`
  mechanism as `open_action_server`.** Its sub-calls use different wording
  than open's, though, so most still fall through to STAGE_FAILED rather
  than a specific code -- the classifier now sees the reason (it lands in
  `detail`), it just does not always recognise it as one of the named
  faults yet. Two of pour's phrasings already matched an existing rule
  without any change: a short Cartesian plan is PLAN_FAILED (both servers
  say "only reached X of the path"), and `_still_holding_bottle`'s "lost:
  fingers have closed to..." is now folded into GRASP_LOST. Widening the
  rest is real, undone work, added rule by rule as pour actually produces
  the message rather than guessed in advance.
- **`Pendant.dispatch()` has no structured result at its own boundary** --
  it prints, and callers (the browser pendant, now this API) capture
  stdout and read it back. `bartender_api/movement.py` pre-validates
  everything it can (axis name, jog bound, gripper bound, point existence)
  so dispatch only ever runs with something it should accept; what is left
  is classified by matching the two text shapes Pendant's call sites
  actually produce (`_report`'s "FAILED: ..." and `_require_pose`'s
  "cannot read ... through /compute_fk"). Verified against a real jog that
  failed for a real reason (`only 0.00 of the path was reachable`) and a
  real one that succeeded, both against the running sim, not just a mock.

A single HTTP/JSON surface so that something which is not a ROS node — a
VLM, a planner, a phone, a test harness — can find out what is on the bar,
ask whether a thing is possible, and make the robot do it.

## Using it

Start the sim, wait for `You can start planning now`, then run the server:

```bash
cd ros2_ws && source install/setup.bash
ros2 run bartender_api server                    # binds 127.0.0.1:8090
```

Everything below is a real, working route today — not the sketch further
down, which also includes what is not built yet.

**Look around, read-only:**

```bash
curl http://127.0.0.1:8090/world
curl http://127.0.0.1:8090/state
```

**Ask before acting:**

```bash
curl -X POST http://127.0.0.1:8090/can \
  -H 'Content-Type: application/json' \
  -d '{"verb": "pour", "args": {"bottle": "whiskey", "glass": "glass"}}'
```

**Move something.** `point` must be one already taught (`ros2 run
bartender_teach teach` then `list` shows what exists — `home`, `b_home`,
`whiskey_approach`, `cola_approach` out of the box). `axis` is `j1`..`j6`
(degrees), `x`/`y`/`z`/`tx`/`ty`/`tz` (mm) or `rx`/`ry`/`rz` (degrees), same
as the terminal pendant's own `jog` command:

```bash
curl -X POST http://127.0.0.1:8090/move/point \
  -H 'Content-Type: application/json' \
  -d '{"arm": "a", "point": "whiskey_approach"}'

curl -X POST http://127.0.0.1:8090/move/jog \
  -H 'Content-Type: application/json' \
  -d '{"arm": "a", "axis": "j1", "amount": 5}'

curl -X POST http://127.0.0.1:8090/gripper \
  -H 'Content-Type: application/json' \
  -d '{"arm": "b", "position": 0.25}'
```

Every `/move/*` and `/gripper` response is `{"ok": bool, "message": "..."}`
-- `message` is the pendant's own captured output, not a code, because that
is genuinely all `Pendant.dispatch()` produces (see the note on this under
"Three things worth knowing" above). A busy server answers
`{"ok": false, "message": "busy: a command is already running"}` with HTTP
409 rather than queuing the request.

**Pick a bottle.** `GET /bottles` lists what can be picked, and
`POST /pick {"bottle": "whiskey"}` runs the `grab_whiskey` pipeline taught on
the pendant, start to finish. A bottle is pickable exactly when the point
file has a `grab_<bottle>` pipeline, so adding one is teaching it, not code.
The answer is `{"ok": bool, "bottle": ..., "message": ...}`: 200 when the
whole pipeline finished, 409 when it was refused or stopped partway. For
the one-arm workcell, start the server with `--points workcell`; see
[WORKCELL.md](WORKCELL.md#picking-a-bottle-through-the-api).

```bash
curl http://127.0.0.1:8090/bottles
curl -X POST http://127.0.0.1:8090/pick \
  -H 'Content-Type: application/json' -d '{"bottle": "whiskey"}'
```

**Make a drink.** `GET /drinks` lists the menu, and says for each drink
whether all its scripts are taught. `POST /make {"drink": "whiskey_cola"}`
runs that drink's scripts (pendant pipelines) in order as one command. It
stops at the first one that does not finish and answers with `stopped_at`.
The menu is a YAML file (`--menu`, which defaults to the `--points` name:
`--points workcell` reads `workcell_menu.yaml`). A drink with untaught
scripts or missing points is refused before anything moves. See
[WORKCELL.md](WORKCELL.md#making-a-drink-through-the-api).

**Ask in plain words.** `POST /ask {"text": "a jack and coke please"}`
tells you which call a sentence means, and **runs nothing**. The caller, or
a person, sends the call it returns:

```json
{"ok": true, "action": "make", "confidence": 0.89,
 "request": {"method": "POST", "path": "/make", "body": {"drink": "whiskey_cola"}}}
```

It asks TypeSafe's Jev, through OpenRouter, in one call of about 0.3–0.5 s.
The first call can take about 2 s. The text is sent to OpenRouter, and
`JEV_KEY` must be set, or the route answers 503. It picks one of
`/world`, `/drinks`, `/make` (a drink from the live menu) or `/pick` (a
taught bottle), or refuses. A pick below 0.6 confidence
(`intent.MIN_CONFIDENCE`) is answered 422 with `why` (for example "please
rephrase" or "which drink?"), never with a guess. A drink on the menu
whose scripts are not all taught is also refused ("not ready yet"), since
`/make` would only refuse it. `/can` is not offered
yet, because it needs a station and a glass that free text rarely names.

**From another machine on the same LAN:** bind to this host's LAN address
instead of localhost, e.g. `ros2 run bartender_api server --host
192.168.1.155`, and use that address in place of `127.0.0.1` above. The
server prints a warning when it does this, because it means: **from that
point on, `/move/*` and `/gripper` have no authentication** -- anyone who
can reach that address and port can jog the arms. See "Safety". Only bind
wide on a network you trust, and prefer a VPN (Tailscale/WireGuard) or an
SSH tunnel over a router port-forward if the other machine is on a
*different* network -- a direct forward puts an unauthenticated
robot-control endpoint on the open internet, which this project's own
Safety section says plainly not to do without adding at least a shared
token first (not built).

## Why a separate layer at all

Three reasons, in order of how much they matter:

1. **A VLM cannot speak ROS.** rclpy in the model's process is not an
   option, and a DDS client on the far side of a network is worse.
2. **Callers need to ask before acting.** A planner that discovers
   reachability by issuing a goal and watching it fail is a planner that
   knocks bottles over. The geometry needed to answer "can you?" already
   exists in `layout.py` and is not exposed anywhere.
3. **It pins a contract.** The skills' internals churn; an agent built
   against them should not have to.

## What already exists to build on

This is deliberately thin — most of it is assembly, not invention.

| need | what exists today |
|---|---|
| where things are, who can reach them | `bartender_open/layout.py`: `STATIONS`, `BOTTLE_SLOTS`, `free_slots()`, `servicing_arms()`, `APPROACH_WINDOW` |
| live object poses | `/world/bar_world/dynamic_pose/info` (bridged, 60 Hz, every non-static model) |
| composite skills | `PourDrink`, `OpenBottle` actions, with feedback and measured results |
| primitive motion, with bounds and refusals | `bartender_teach.Pendant.dispatch()` — jog limits, branch wrapping, two-arm frame safety, one-command-at-a-time locking |
| named poses and sequences | `bartender_teach` points and pipelines (`config/taught_points.yaml`) |
| an HTTP server pattern with no dependencies | `bartender_teach/teach_gui.py` — stdlib `http.server`, localhost by default |

**The movement API should wrap `Pendant.dispatch()`, not reimplement it.**
The pendant already encodes every bound and refusal that stops a typo from
becoming a collision, and the browser GUI proves the pattern: it owns no
robot logic, so a fix reaches every front end at once. A second motion
implementation would be a second set of bugs.

## Four surfaces

### 1. World — "what is there, and who can reach it"

```
GET /world
```

```json
{
  "frame": "world",
  "counter": { "centre": [0.08, 0.0], "size": [1.76, 1.60], "top_z": 0.9 },
  "arms": [
    { "id": "a", "origin": [-0.45, -0.40, 0.9], "yaw": 0.0,
      "serves_world_y": [-0.30, 0.10], "skills": ["pour", "hold"] },
    { "id": "b", "origin": [0.61, 0.40, 0.9], "yaw": 3.14159,
      "serves_world_y": [-0.10, 0.30], "skills": ["open", "hold"] }
  ],
  "stations": [
    { "id": "whiskey", "kind": "bottle", "xy": [0.08, -0.30], "slot": -2,
      "reachable_by": ["a"], "occupied": true,
      "pose": { "xyz": [0.08, -0.30, 0.900], "tilt_deg": 0.2,
                "source": "sim_ground_truth", "confidence": 1.0,
                "stamp": "2026-09-19T21:02:11Z" } },
    { "id": "glass", "kind": "vessel", "xy": [0.20, -0.55],
      "reachable_by": ["a"], "occupied": true, "pose": { "...": null } },
    { "id": "slot_+1", "kind": "empty_slot", "xy": [0.08, 0.15],
      "reachable_by": ["b"], "occupied": false, "pose": null }
  ]
}
```

Three things this must get right:

- **`reachable_by` is computed, never written down.** It comes from
  `layout.servicing_arms()`, so it cannot disagree with the robot.
- **Empty slots are stations too.** "Where could I put this down" is a
  question a planner will ask, and the free slots are surveyed positions,
  not leftover tabletop.
- **Every pose carries `source` and `confidence`.** Today the source is
  simulator ground truth and confidence is 1.0. When perception replaces it
  the field already exists, and callers written against it do not change.

#### Camera perception (`server --perception ...`)

Where bottle poses come from is a startup choice, reported back as the
top-level `"perception"` field. There is no silent fallback between them:

| mode | bottle stations | other stations |
|---|---|---|
| `ground_truth` (default) | sim ground truth, as above | sim ground truth |
| `camera` | stand depth camera only; no data means `unknown` | `unknown` -- nothing looks at them yet |
| `compare` (sim only) | camera, plus `ground_truth_xy` and `error_mm` | sim ground truth |

`compare` refuses to start without the Gazebo pose topic, and `camera` never
reads it, so a hardware run cannot quietly be answered by a simulator.

Only `kind: "bottle"` stations are observed. Each gets:

```json
{ "id": "cola", "kind": "bottle", "xy": [0.08, -0.15],
  "observation": "observed", "occupied": true,
  "in_place": true, "offset_mm": 3.1,
  "pose": { "xyz": [0.0812, -0.1529, 0.9], "source": "camera",
            "confidence": 0.81, "fit_rms_mm": 2.4 },
  "reason": null, "frame_age_s": 0.12,
  "valid_px": 2210, "foreground_px": 1035, "confidence": 0.81 }
```

- **`observation`** is `observed`, `missing` or `unknown`, and the
  difference between the last two is the point. `missing` (`occupied:
  false`) needs positive evidence: valid depth that passed *through* the
  space the bottle would fill. `unknown` (`occupied: null`, `in_place:
  null`) is everything else: no frame, a frame older than 1 s, a calibration
  frame that does not match `CameraInfo`, distorted or unsupported depth,
  depth holes (dark, glass and shiny surfaces are stereo-poor), or something
  in front. `reason` says which. **Treat `null` as "don't know", never as
  "empty".**
- **`offset_mm`** is the distance from the fitted bottle centre to the
  station's `xy`. The centre comes from fitting that bottle's known
  cross-section (circle for cola and beer, square for the whiskey) to the
  visible body, not from the nearest surface.
- **`in_place`** is `offset_mm <= TOL_MM`. `TOL_MM` is set from *measured*
  error (`scripts/perception_error.py`, then repeated placements on the real
  bar), comfortably above the p99 -- not from a depth-sensor datasheet.
  Until that has been measured it is a 15 mm placeholder.
- **`confidence`** is `min(1, foreground_px / px a full body would cover)`,
  reduced by how badly the shape fit (`fit_rms_mm`). It is a coverage and
  fit-quality score for ranking and thresholds, **not a probability**.
  `missing` reports 1.0: the see-through evidence was sufficient.

Calibration is `T_world_optical`, the pose of the camera **optical** frame
(z forward, x right, y down) in world, mapping optical points to world, in `bartender_api/config/stand_camera.yaml`, checked against
`CameraInfo.header.frame_id` on every frame. The sim copy is derived from
the camera model in `bar_world.sdf`; on hardware,
`scripts/calibrate_stand_camera.py` writes one and `--camera-config` points
the server at it.

#### Which bottle it is (`label`)

In `camera` and `compare` modes, each occupied bottle station also carries a
`label` field, when PaddleOCR is installed or Gemini is configured
(`GEMINI_API_KEY` and `GEMINI_VISION_MODEL`):

```json
"label": { "brand": "Jack Daniel's", "type": "whiskey", "volume_ml": 700,
           "confidence": 1.0, "source": "inventory", "reason": null,
           "read": { "brand": "", "type": "whiskey", "volume_ml": 700,
                     "abv_percent": 40.0,
                     "text_read": "JACK DANIEL'S Tennessee WHISKEY 40% VoL. 70c" } }
```

The station's box is cropped from the colour image, and PaddleOCR and Gemini
both start on it at once (`bartender_api/drink/label.py`):

1. **PaddleOCR, offline, wins when it is sure.** It reads the label text, and
   regexes pull out the type (words like `WHISKEY`, `VODKA`/`WÓDKA`, `GIN`,
   `ジン`), the volume (`70cl`, `500 ml`, `0,75 l`) and the ABV (`40% vol`).
   No pattern says which word is the brand, so the brand can only come from
   the inventory. If the text names a stocked bottle, that is the answer at
   once and Gemini's is thrown away. The crop is enlarged 2× first; each
   bottle takes about 1.5–3.5 s on CPU.
   If the text names no stocked brand outright, one more quick check picks
   a brand from it before waiting on Gemini. That check is TypeSafe's Jev,
   through OpenRouter, when `JEV_KEY` is set (70–500 ms per call; the text
   is sent to OpenRouter). Otherwise it is `label.fuzzy`, a local string
   match that takes about 2 ms. A pick counts only at confidence ≥ 0.8, and
   its `source` is `jev` or `fuzzy`.
2. **Otherwise Gemini's answer**, unless it is unknown. It has been running
   since the start, so the wait is about Gemini's own time, not OCR's plus
   Gemini's. A Gemini request is cut off after 15 s (`vlm.TIMEOUT_S`).
3. **OCR's partial answer** stands when Gemini is off, fails, times out or
   is over quota. For example, a type read from the label with no brand.

Running both costs **one Gemini request per bottle read**, even when OCR
alone would have been enough. The free tier allows 20 requests a day per
model, which a demo can use up; a failed Gemini call still leaves OCR's
answer.

Every reading is then checked against the bottle inventory,
`bartender_api/config/bottles.yaml` (`server --bottles` points at another
one). Both readers misread stylised and Japanese brand names, and a volume
read can change between runs, so a known bottle's fields always come from
the inventory:

| Case | `source` | `confidence` | Fields from |
|---|---|---|---|
| Brand or one of its `aliases` in the text read (or Gemini's brand a close spelling match, at least 0.8) | `inventory` | 1.0 (or the match score) | the inventory row |
| OCR text names no brand outright; `label.fuzzy` finds a stocked brand in its words | `fuzzy` | the similarity score, 0.8–1.0 | the inventory row |
| Fuzzy found none; Jev (`JEV_KEY` set) picks a stocked brand from the OCR text | `jev` | Jev's confidence, 0.8–1.0 | the inventory row |
| Gemini names a brand not in the inventory, and that brand is in the text it read | `gemini` | 0.5 | Gemini's reading |
| Brand not read; the type spelled on the label matches exactly one row | `inventory` | 0.6 | that row |
| Brand not read; type spelled on the label, not in the inventory or in several rows | `ocr` or `gemini` | 0.4 | the reading, with `brand: null` |
| Nothing usable read | none | 0.0 | `type: null` |

- **`type: null`** means don't know, as with `unknown` elsewhere. `reason`
  says why, and `read` shows what the label said. `brand: null` with a type
  means the kind of bottle is known but not the brand.
- A brand or type counts only when it appears in the text read. From
  straight above, Gemini named a brand with no label text in view, and that
  is dropped.
- A misread brand whose wrong text Gemini also reports as read can come back
  as a new bottle with the right type and the wrong brand. Adding that
  misreading to the row's `aliases` fixes it.
- **`volume_ml`** is what the bottle holds when full, not what is left in it.
- A label is read **once per bottle**: kept until the station is seen empty,
  and a failed read is retried every 30 s. A first read can hold `/world`
  for a few seconds per bottle.
- On phone photos of the bar, OCR alone named Jack Daniel's and Tenjaku, but
  read nothing from the small Żubrówka label, which needs Gemini.
  `scripts/read_labels.py PHOTO --box u0,u1,v0,v1 [--no-gemini]` runs the
  same reading on a photo (phone `.dng` included).
- The Docker image downloads the OCR models when it is built
  (`PADDLE_PDX_CACHE_HOME=/opt/models/paddlex`), so OCR needs no network at
  run time.

### 2. Feasibility — "could you?"

```
POST /can      { "verb": "pour", "args": { "bottle": "gin", "glass": "glass" } }
```

```json
{
  "ok": false,
  "reasons": [
    { "code": "OUT_OF_APPROACH_WINDOW",
      "detail": "gin is at world y=+0.15; arm A serves [-0.30,+0.10]",
      "measurements": { "station_y": 0.15, "window": [-0.30, 0.10] } },
    { "code": "NO_ARM_CAN_DO_BOTH",
      "detail": "arm B reaches gin but the glass is 1.035m from its base (limit 0.85)",
      "measurements": { "glass_range_from_b": 1.035, "reach_limit": 0.85 } }
  ],
  "alternatives": [
    { "verb": "pour", "args": { "bottle": "whiskey", "glass": "glass" } }
  ]
}
```

No motion, no planning-time cost beyond geometry. This is the endpoint a
VLM should call on every candidate action before committing to a plan, and
the one that makes the difference between an agent that reasons about the
bar and an agent that flails at it.

`alternatives` is optional and cheap here (same-kind stations that *are*
feasible). It is worth including: it turns a refusal into a next step.

### 3. Movement — primitives, for when no skill fits

```
GET  /state                     joints, tool pose, gripper, per arm
POST /move/point   { "arm":"a", "point":"whiskey_approach" }
POST /move/joints  { "arm":"a", "joints":[...], "speed":0.3 }
POST /move/tool    { "arm":"a", "xyz":[0.53,0.10,0.12], "quat":[...],
                     "frame":"base_link", "collision_checked":true }
POST /move/jog     { "arm":"a", "axis":"tz", "mm":-40 }
POST /gripper      { "arm":"b", "position":0.25 }
```

Rules, inherited from the pendant and non-negotiable for an autonomous
caller:

- **Frames are explicit.** `frame` is required on `/move/tool`; there is no
  default. Arm B's axes point the other way and a silent default is the bug
  this project keeps finding.
- **Bounds are refused, never clamped.** `jog z 500` when you meant `50` is
  a typo, and clamping turns it into a move that quietly does something
  else.
- **One motion at a time, refused not queued.** Two motion goals interleaved
  on one arm is the failure the pendant's lock exists to prevent.

### 4. Skills and pipelines — the useful verbs

```
GET  /skills                     what exists, with argument schemas
POST /do    { "verb":"open", "args":{ "bottle":"beer", "stow_after":true } }
GET  /job/{id}
POST /job/{id}/cancel

GET  /pipelines                  taught sequences, from taught_points.yaml
POST /pipelines/{name}/run       { "dry": false }
```

`/do` is async and returns a job immediately, because a pour takes minutes:

```json
{ "job": "j-17", "verb": "open", "state": "running", "progress": 0.12,
  "stage": "gripping the opener" }
```

`GET /skills` returning argument schemas is what lets a VLM use the robot
without being retrained when a skill is added. Generate it from the
`.action` files rather than writing it twice.

**Pipelines are the cheap extensibility.** A human teaches a sequence on the
pendant, and it becomes a callable verb with no new code. See
`ros2_ws/src/bartender_teach/README.md`.

## The error taxonomy — the part that matters most

This is the highest-value piece of the whole proposal and it is mostly
mechanical to build, because the measurements already exist.

Today a failure comes back as prose:

> `the beer moved 6.0mm while being pushed on`
> `arm B could not pick up the opener`
> `the opener is not seated on the cap: 280.7mm down (needs 14) and 344.9mm off centre`

Excellent for a human reading a log. Unusable for a planner, which cannot
tell "try again" from "never going to work" from "something is broken".
Every failure should carry a code, the stage it happened in, the numbers,
and whether retrying is sensible:

```json
{
  "state": "failed",
  "error": {
    "code": "OBJECT_NOT_SEATED",
    "stage": "pressing the opener onto the cap",
    "detail": "280.7mm down (needs 14) and 344.9mm off centre (allows 6.5)",
    "measurements": { "depth_mm": 280.7, "depth_required_mm": 14,
                      "offset_mm": 344.9, "offset_allowed_mm": 6.5 },
    "retryable": true,
    "suggest": "re-home both arms and retry; three consecutive failures mean restart the stack"
  }
}
```

A starting set, drawn from failures this project has actually produced:

| code | means | retryable |
|---|---|---|
| `UNKNOWN_STATION` | no such bottle/glass/slot | no |
| `OUT_OF_APPROACH_WINDOW` | geometry says no arm can side-grasp it | no |
| `OUT_OF_REACH` | past the arm's radius | no |
| `NO_ARM_CAN_DO_BOTH` | one arm reaches A, another reaches B, neither both | no |
| `ALREADY_OPEN` | premise wrong; the cap is off and can't re-attach in sim | no |
| `PLAN_FAILED` | MoveIt found nothing | yes |
| `MOVE_STOPPED_SHORT` | controller said success, flange is elsewhere | yes |
| `GRIPPER_NOT_FOLLOWING` | goal accepted, joint never moved | no, restart the sim |
| `GRASP_STOPPED_WIDE` | fingers closed fine, then met the wrong thing | yes |
| `GRASP_LOST` | had it, dropped it mid-sequence | yes |
| `OBJECT_NOT_SEATED` | got there, geometry check failed | yes |
| `OBJECT_DISTURBED` | the workpiece moved more than allowed | yes |
| `SCENE_STALE` | no model poses; the bridge is down | no |
| `STAGE_FAILED` | a named stage failed with no specific reason reaching the classifier | yes |
| `UNCLASSIFIED` | a message the classifier does not recognise at all | no |

The last two are additions from building the classifier, not in the
original thirteen above: a wrong code is worse than an honest "do not
know" (the same reasoning behind the `GRIPPER_NOT_FOLLOWING` /
`GRASP_STOPPED_WIDE` split below), so a coarse stage summary becomes
`STAGE_FAILED` and anything genuinely unrecognised becomes `UNCLASSIFIED`
rather than a guessed specific code.

`MOVE_STOPPED_SHORT` and `GRIPPER_NOT_FOLLOWING` are not hypothetical —
see `docs/ROADMAP.md`, where both are written up as defects that have since
been fixed. An agent that cannot distinguish them from "the plan was bad"
will retry forever.

`GRIPPER_NOT_FOLLOWING` and `GRASP_STOPPED_WIDE` are separate codes on
purpose, and that separation was itself a bug fix: one is the gripper and
is not retryable, the other is the arm or the workpiece and is. They used
to share a message, and it sent one investigation to the wrong half of the
robot.

## Safety

The page moves a robot arm. `teach_gui` already takes the right line and
the API should copy it exactly:

- **Bind `127.0.0.1` by default.** `--host 0.0.0.0` is reasonable on an
  isolated robot LAN and a bad idea anywhere else; print a warning. Done,
  and the warning text now says the process moves the robot rather than
  the older "read-only today" wording.
- **No authentication is not a plan.** Before this is exposed to anything
  off-host, a shared token in a header is the minimum. Not built.
- **Every job cancellable**, and a global `POST /stop`. Not built --
  `/move/point` and `/move/jog` block for the length of one motion and
  there is nothing to cancel mid-flight yet, which is acceptable for a
  single jog but will not be once `goto` targets are longer transits.
- **`dry` on everything that moves**, so a planner can rehearse. Not
  built.
- **Rate-limited and serialised. Refuse a second motion, do not queue
  it.** Done for serialisation: `MovementBridge` holds the same
  non-blocking-acquire-or-refuse lock `teach_gui.py`'s `Bridge` does, so a
  second `/move/*` or `/gripper` call while one is running gets `{"ok":
  false, "message": "busy: ..."}` rather than queuing. It only serialises
  within this one process, though -- it does nothing about the browser
  pendant or the terminal pendant issuing a goal to the same arm at the
  same time, which is a real, pre-existing gap this does not close. Not
  rate-limited.

An autonomous caller should also be given a budget — maximum actions per
minute, maximum consecutive failures before it must stop and ask — enforced
server-side. A VLM in a retry loop against a robot is the failure mode worth
designing against from the start.

## Transport: HTTP now, MCP later

HTTP/JSON first, because `teach_gui` shows it costs nothing (stdlib only,
no CDN, works with the network down) and because every client can speak it.

An **MCP server** is the natural second front end once the verb set settles:
the tool-per-verb shape maps onto `/skills` almost exactly, and it would let
a model call the robot without bespoke glue. Do not build it first — the
verb set and the error taxonomy are what need to be right, and those are
easier to iterate over plain HTTP.

## Build order

1. ~~`GET /world` and `GET /state` — read-only, no risk, immediately useful
   for prompting a VLM.~~ **Built**, in `bartender_api`.
2. ~~`POST /can` — pure geometry over `layout.py`.~~ **Built.**
3. ~~The error taxonomy, applied to the two existing skills.~~ **Built for
   both.** `open_bottle` via `Arm.last_error`/`_why`/`_why_any`,
   `pour_drink` via `PourActionServer.last_error`/`_why`. Most of
   `pour_drink`'s specific reasons still classify as STAGE_FAILED rather
   than a named code, because its wording differs from `open_bottle`'s and
   the rules were written against real messages, not guessed in advance --
   see `errors.py`'s module docstring for which two already match. Widening
   that coverage can happen incrementally; it does not block step 4 below.
4. `POST /do` + jobs over the existing actions.
5. ~~Movement primitives over `Pendant.dispatch()`.~~ **Partly built, out
   of order:** `POST /move/point`, `POST /move/jog`, `POST /gripper` exist
   (`bartender_api/movement.py`), by explicit request to have "simple
   movement" before `/do`. `/move/joints` and `/move/tool` (arbitrary
   absolute targets) are not built -- Pendant has no verb for either, and
   adding them means either a new Pendant command or bypassing it and
   losing its bounds, neither of which is "simple". A global stop and an
   action budget (see "Safety") are also not built yet; today's only
   safety net is the one-command-at-a-time lock and Pendant's own refusals.
6. Pipelines.
7. MCP front end.

Steps 1–3 were worth doing on their own even before anything autonomous
arrived: building `/can` and the error taxonomy caught two real bugs
(`layout.servicing_arms()` misapplied to the glass and the opener holster,
and `open_bottle`'s coarse failure messages losing the specific reason a
sub-call had already logged) before they shipped, exactly the kind of
thing this paragraph predicted they would.
