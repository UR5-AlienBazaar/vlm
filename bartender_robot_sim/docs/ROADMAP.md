# Roadmap

Where the project is, what is blocking it, and what is worth building next.

Dates are deliberately absent. The ordering and the exit criteria are the
useful parts.

## Where it is now

Working, in simulation, measured:

- **Pour** — `PourDrink` makes a whiskey and coke end to end: 45 ml spirit,
  135 ml mixer, both bottles returned upright into their wells. The
  whiskey half is reliable; the cola half is not (defect 4).
- **Open** — `OpenBottle` takes the cap off a beer with both arms, one
  holding and one pressing. It has done so, and it is **not** reliable
  yet (defect 3).

Both have completed; neither completes every time. Read "what is blocking
autonomy" below before assuming a green run means much — a single run is
weak evidence here, deliberately.
- **Teach pendant** — terminal and browser, two arms, tool centre points,
  taught points, and recorded pipelines.
- **1091 tests**, none of which need a robot.
- Two recordings of successful runs in `recordings/`.

## What is actually blocking autonomy

Be honest about these before planning anything clever on top. An agent
cannot compensate for a robot that reports success when it has not moved.

### 1. Moves stop short while the controller reports success — **FIXED**

**Was: severity highest.** MoveIt computed a Cartesian path at **100 %**,
`b_ur_arm_controller` reported *"Goal reached, success!"*, and the flange
was **267 mm** from where it was sent. The only thing that noticed was the
skill's own `arrived_at` check.

**Cause, confirmed.** The arm controllers set only `goal_time` and
`stopped_velocity_tolerance`. `joint_trajectory_controller` defaults every
*per-joint* tolerance to 0.0, and 0.0 means **not checked** — so the goal
check passed trivially and the controller reported success wherever it
stopped. Reproduced cheaply by driving arm A's wrist 250 mm into the bar
top: `SUCCEEDED`, `error_code 0`, `wrist_1` **0.756 rad** from its goal.

**Fix.** Per-joint `goal: 0.01` on both arm controllers
(`bartender_description/config/controllers.yaml`). Chosen from measurement,
not taste: free-space moves from 4 s down to 0.6 s settle to **≤ 0.0009
rad**, the jam reached **0.756** — so it sits ~11× above the worst honest
error and ~75× below the failure. The same blocked move now returns
`ABORTED`, `error_code -5` (`GOAL_TOLERANCE_VIOLATED`).

**No `trajectory:` tolerance, deliberately.** That polices
|desired − actual| *during* a move. Measured: free moves peak at 0.0045 (4 s)
to 0.0299 (0.6 s), but a real pour reaches **0.74** on the elbow and
recovers, and driving back *out* of a jam peaks at **1.95** — the recovery
move would be the one it aborted.

**What it exposed.** Four moves genuinely cannot arrive, because they end
by setting something down or touching it, and they had been relying on the
controller's silence. Each now declares `may_stall` where a reader can see
it: the pour's bottle place, the opener onto its post, the beer into its
stand, and the bell onto the cap. Measured, the pour's place ends with
`wrist_1` held **0.029 rad** (~6 mm at the bottle) short of its command,
steady — the arm pressing a bottle onto a counter already holding it up.
Every transit and every pick descent is still checked, which is where the
267 mm failure was.

Guarded by `bartender_description/test/test_controller_limits.py`.

**Verified:** pour 45 ml + 135 ml full drink; open cap off at 330 mm with
the bottle moving 5.9 mm, including the stow.

### 2. Grippers stop following — **FIXED**

**Was: the top defect.** Goals accepted (`ok=True`), joint never moves,
on both arms, for the rest of the simulator run.

**Cause, measured.** The knuckle joint is `limit=[0.0, 0.8]` and "open the
gripper" asked for exactly 0.0. A knuckle left **at rest on its lower
limit** stops being driven. Two arms on one stack, so each row shares a
simulator with the row beside it:

| parked at | dwell | then commanded 0.50 |
|---|---|---|
| 0.300 | 90 s | moved, 2.5 s |
| 0.020 | 90 s | moved, 5.0 s |
| 0.020 | 300 s | moved, 5.6 s |
| 0.000 | 10 s | **never moved** |
| 0.000 | 90 s | **never moved** |
| 0.000 | 300 s | **never moved** |

The **upper** limit is harmless — 0.800 for 90 s then moved in 3.5 s — so
only the open side needs a margin.

**The old exit criterion was "say whether it is a sim artefact or a
controller-configuration bug". It is a sim artefact.** The controller
stays `active` and goes on accepting goals; Gazebo's own link poses on
`/world/bar_world/dynamic_pose/info` agree the fingertip is stationary to
four decimals; `/clock` holds a steady 0.21 real-time factor across a
120 s window in which nothing moves; the arm's own six joints keep
tracking; and deactivating and reactivating arm B's gripper controller
left its knuckle exactly as dead as before. What *does* bring the joint
back is **disturbing** it: moving arm B's wrist, while the knuckle was
dead with a stale 0.5 command pending, left the knuckle at 0.500. A joint
that resumes when something shakes it was not being integrated, which is
a physics-side state and not something ros2_control has a setting for.

Not fully explained: the arm also moves between the open and the grasp in
the sequences that originally failed, and there the gripper stayed dead.
So "any motion revives it" is too strong — it has been seen once. The
avoidance does not depend on knowing which.

**Fix.** `GRIPPER_OPEN_POS = 0.02`, written as limit + margin, in all
three packages that command a gripper; anything outside `0.02..0.8` is
refused rather than clamped. It costs 1.8 mm of pad opening — 83.2 mm
instead of 85.0 — which matters only to the whiskey, whose run-in
clearance drops from 3.9 to 3.0 mm a side. That is a real 23 % cut in the
tightest tolerance in the system and it is written down where the
constant is, not waved away.

**Residual, known and measured.** The joint still *brushes* the stop on a
full-speed open: traced 0.0178 → −0.0000 → 0.0200, 0.07 s of simulated
time, because the ramp outruns the joint and it arrives at 0.5 rad/s. A
brush has never been seen to matter — the same gripper grasped the opener
cleanly twenty seconds later, and every death measured involved ten
seconds or more *at rest*. `command_gripper` now fails loudly if the
fingers are **left** there, so the next occurrence is named at the moment
it happens rather than minutes later.

**Also fixed, and it is why this took three runs to confirm.** `grasp`
reported "the gripper is not following" for two unrelated faults: a joint
that never moved (the gripper), and fingers that closed perfectly well
and then stopped on something too wide (the arm, or the workpiece). One
run stopped with the pads 49.1 mm apart on a 24.0 mm shaft after closing
0.345 rad, and the message blamed the gripper. The two now read
differently.

Guarded by new tests in `bartender_open/test/test_grip.py`,
`bartender_teach/test/test_pipelines.py` and `test_teach_points.py`,
including a cross-package check that the three copies of the constant
cannot drift.

### 3. The open is unreliable — **now the top defect, partially fixed**

With 1 and 2 fixed, what is left is its own problem rather than a
symptom. **Three opens before this fix, 0 of 3 complete — but no gripper
freeze in any of them**, and the grasping is the reliable part:

| run | opener grasp | beer grasp | failed at |
|---|---|---|---|
| 1 | stopped 49.1 mm wide | — | the descent did not land where it thought |
| 2 | 0.611 rad, 2.3 mm bite | 0.475 rad, 1.6 mm bite | bell seated 110.8 mm from the cap, 114.8 mm off centre |
| 3 | 0.629 rad, 4.3 mm bite | 0.478 rad, 1.9 mm bite | bell seated 12.9 mm of 21, 2.7 mm off centre; arm A then lost the beer |

Five of six grasps succeeded and every one of them reported a plausible
bite. The failures were all in the **press**: where arm B puts the bell,
and whether arm A can hold the beer while it is pushed on.

Note that run 1's failure was mis-labelled as a gripper fault by the old
message; see defect 2. That is the kind of thing that made this defect
hard to see the shape of.

**Found and fixed: the bell was being aimed at a swinging target.** Arm A
holds the beer by friction on its neck and lifts it 50mm; that bottle
keeps swinging on the pads for real seconds after the lift's own
trajectory reports done -- traced at 63mm of drift in x over about 6s.
`_press` already re-aimed before every descent, but a re-read taken
mid-swing is not the same as a settled one, and the descent it feeds
takes long enough that the target has moved again by the time the bell
arrives. That is what run 2's 114.8mm-off-centre press looks like.

The fix is `_wait_for_beer_to_settle` in open_action_server.py: poll the
beer's pose until it holds within 3mm across a full second (measured
against the traced swing, with margin) before aiming, both on the first
approach and before every retry. Polled rather than a fixed sleep,
because the swing's size depends on how the lift disturbed the grip and
varies run to run -- see BEER_SETTLE_MOVE for the reasoning. 4 new tests,
mutation-verified.

**Traced once since the fix (run "dfix_p2"): the catastrophic miss is
gone, a smaller one is not.** `beer settled after 6.2s` confirmed the
swing is real and multi-second, exactly as measured. But the first press
attempt still landed 12.8mm off centre against a cap that had been sitting
motionless the whole time (checked directly against its own logged pose,
unchanged across the descent). The chamfer partially corrected it, down
to 5.9mm off centre on the second attempt (inside SEAT_OFFSET_MAX), but
the bell only reached 6.9mm of its 21mm travel.

A first guess -- recorded here and then disproved, because it would have
sent the next investigation the wrong way -- was that this was arm B's
own Cartesian execution missing its target. It was not. Three direct
diagnostics against the running sim (`press_precision.py`,
`opener_grip_check.py`, `loaded_precision.py`, all in the scratchpad,
none committed) checked it out from every angle: the bare flange lands
within 1mm of the exact xy that failed; the opener sits within 2mm of
what OPENER_GRIP_Z predicts, statically; it does not swing on its own
grip during the transit; and even carrying the real opener, driven
straight at that exact failed xy with no beer or cap involved at all, the
opener's own ground-truth pose landed 1.6mm from the target. Arm B's
kinematics are not the problem.

**What is actually happening: the beer does not just swing, it can hang
at a sustained, undamped tilt, and `_wait_for_beer_to_settle` cannot wait
that out because there is nothing to wait for.** Found by adding
orientation logging (`pose_log2.py`) alongside position, because the
position-only trace could not distinguish "still moving" from "moving in
a circle around a level rest point." A clean traced run ("clean1") shows
it directly: after the lift the bottle's tilt does not decay to zero, it
settles into a periodic oscillation between roughly 2.3 degrees and 6.3
degrees, in lock-step with a ~30mm swing in x, that is *still going after
15 seconds* -- long enough that `_wait_for_beer_to_settle` hit its own
timeout (`beer had not settled within 15s; aiming at it anyway`, logged
exactly as designed) and aimed at whatever the swing gave it. The press
then landed 50.8mm off centre, and continued contact with the swinging
bottle drove the tilt as high as 31 degrees before the sequence gave up.
A tilted cap presents a non-horizontal top surface to a bell that only
ever descends straight down, so even a correctly-centred xy aim can catch
the cap's edge first -- which is a second, independent way to miss beyond
simply aiming at a stale position, and no amount of extra settle-margin
fixes it if the "settled" state itself is a moving average.

The three traced+tilt-logged runs line up with how hard arm A's pads bit
into the neck: 1.7mm of bite damped out in ~2s and pressed cleanly
(pre-fix dfix_p1); 2.8mm of bite took 6.2s to mostly damp and left an
11mm residual (dfix_p2); 3.2mm of bite never damped at all inside 15s and
produced the 54.8mm miss above (clean1). More interpenetration in the
grasp looks like a springier, less-damped pivot for the bottle to hang
from -- consistent with an underdamped pendulum, not proof of one. This
is not yet a fix, only a much better-aimed description of the failure:
waiting longer will not help a system that is not decaying, and the next
real candidate is changing the grip or the lift itself (a firmer or
softer squeeze, a slower lift, or actively damping the swing) rather than
reading the clock differently. That needs its own measurement before
committing to one.

One unrelated finding from this session, worth recording so it does not
cost someone else the debugging time it cost here: a fresh diagnostic run
("tilt1") came back with an obviously-broken 365mm offset, identical
across both retry attempts. The cause was not the sim -- it was seven
`watch_tilt.py` processes from an unrelated earlier session, still
running 14+ hours later and loading the machine to 7-8. Killed by PID
(never by `pkill -f` pattern; see CONTRIBUTING.md), and the very next run
on the same, now-idle machine came back with the physically-plausible
54.8mm/17.8mm of "clean1" above. **Check `ps`/`uptime` before trusting a
number that looks too strange to be physical.**

**A first look at GRIPPER_SQUEEZE, isolated from the full open sequence:**
one clean grasp-and-lift trial per value, each on a freshly relaunched
sim so a disturbed bottle from a previous trial can't confound the next
one (`squeeze_vs_swing.py`, scratchpad, not committed):

| GRIPPER_SQUEEZE | peak tilt over 12s | x-range |
|---|---|---|
| 0.10 | 1.68 deg | 7.4mm |
| 0.18 (current) | 5.92 deg | 7.7mm |
| 0.30 | 10.83 deg | 35.7mm |

Monotonic, and in the direction the pendulum-compliance idea predicts:
less squeeze, less tilt. But this is **one trial per value**, and bite
has already shown a lot of run-to-run randomness even at a fixed squeeze
command (2.3 to 4.3mm across otherwise-identical runs) -- so this table
is a lead, not a confirmed dial, until it is repeated enough times per
value to see whether the ordering survives that noise.

It also has not been checked against the reason GRIPPER_SQUEEZE is 0.18
and not something smaller: its own docstring in arm.py records that a
lighter first-contact grip (1.1mm of bite) held through a lift and then
let the bottle slide 21mm the moment the *other* arm pushed on it during
a press -- caught by the gate, correctly, as a lost grip. `still_holding`
read true in all three trials above, but that only checks the clamp
angle did not creep at rest; none of these trials put the press's lateral
load on the grip. Turning squeeze down to fix the swing without
re-checking that is a straight shot at reopening the defect the squeeze
was raised to close in the first place.

**Exit:** three consecutive opens succeed. Not met. Next, in order:
repeat the squeeze sweep enough times per value to trust the ordering,
then check the most promising value still holds under an actual press
load before touching the constant that ships.

### 4. The cola slips out mid-pour

Documented at length in `pour_action_server.py` as the largest remaining
defect in the default path. At the 97° pour angle friction carries the whole
bottle weight; the grip check finds the fingers closed to the full commanded
angle with the bottle gone. Intermittent.

### 5. Smaller, known

- `bartender_open` does not publish the whiskey or the cola into its
  planning scene. The redesign removed the geometry that made it bite; the
  hole remains.
- Gripper commands are deliberately fire-and-forget, so a pipeline `grip`
  step returns while the fingers are still travelling. Use a `wait` step
  after a grip that matters.

## Phases

### Phase A — make the robot honest *(prerequisite for everything)*

Both of the defects this phase existed for are **fixed**.

Defect 1: a move that does not arrive now fails, and the four moves that
legitimately end in contact say so in the code rather than relying on the
controller not looking.

Defect 2: no actuator is parked on a joint limit any more, so the gripper
does not silently stop being driven. The failure it caused was the one
that made the whole system feel unreliable, because a dead gripper
produces a *different* wrong answer at every later stage.

What that buys is exactly what the phase was for: the robot's failures
now point at the thing that failed. Across the three opens run since,
five of six grasps worked and every failure is in the press — a descent
that landed wrong, a bell 110 mm off, a beer pulled out of arm A. All
real, all locatable, none disguised as something else.

**Exit:** three consecutive opens succeed. Not met; see defect 3, which
is now the top defect and is about where arm B puts the bell.

### Phase B — the read-only API — **built**

`GET /world`, `GET /state`, `POST /can` all exist in a new package,
`bartender_api`, tested against the real running sim (1091/1091 tests
pass). The error taxonomy is applied to both skills' failures now
(`Arm.last_error` for `open_bottle`, `PourActionServer.last_error` for
`pour_drink`), though most of `pour_drink`'s still classify as the generic
"a stage failed" code rather than a named one, because its messages are
worded differently from `open_bottle`'s and the classifier's rules are
written against real messages rather than guessed in advance -- see
`docs/CONTROL_API.md`.

No motion, so no risk, and immediately useful: it is what you would put in a
VLM's prompt, and `/can` plus real error codes would have shortened several
past debugging sessions on their own. Building it did exactly that once:
`layout.servicing_arms()` -- built for side-grasping a bottle off the
line -- turned out to say no arm could reach the glass or the opener
holster, neither of which is side-grasped, and `/world`'s `reachable_by`
inherited the same bug before `bartender_api/reach.py` fixed it. Caught by
curling the live sim, not by a unit test, which is now backed by one.

**Exit:** a caller can describe the bar and rule out an impossible action
without touching the robot. Met: both skills' failures now carry a specific
reason through to the result message. What remains is widening the
classifier's rules to recognise more of `pour_drink`'s own wording, which
is incremental and does not block Phase C.

### Phase C — the acting API — **started, simple movement only**

`POST /move/point`, `POST /move/jog` and `POST /gripper` exist in
`bartender_api`, wrapping `Pendant.dispatch()` (`bartender_api/movement.py`)
rather than reimplementing it, so they inherit every bound the pendant
already has: MAX_JOG_MM/MAX_JOG_DEG refused not clamped, the gripper's
refusal to park on its lower limit, `goto` always driving the arm a point
was taught on. Verified against the real running sim: a real jog that
failed for a real reason (`only 0.00 of the path was reachable`), a real
one that succeeded, a real `goto` transit and return, and two concurrent
requests where the second was refused as busy rather than queued.

Deliberately not built: `/move/joints` and `/move/tool` (arbitrary
absolute joint/Cartesian targets -- Pendant has no verb for either),
`POST /do` with jobs and cancellation, pipelines as callable verbs, a
global stop, and an action budget. The one-command-at-a-time lock only
serialises within `bartender_api`'s own process; it does nothing about the
browser or terminal pendant issuing a goal to the same arm at the same
time, which is a pre-existing gap and not new here.

**Exit:** a script that is not a ROS node can make a whiskey and coke. Not
met -- that needs `/do` and pipelines, neither of which exist yet.

### Phase D — more bar

The open question from the layout work: **more bottles.** The line holds
five slots at 0.15 m pitch, three filled, two free — and the two free ones
are on arm B's side, which cannot reach the glass. Three options, in
increasing order of work:

1. **Two spirits in the free slots.** Arm B can pick them; nothing pours
   them. Smallest change, least useful.
2. **Give arm B its own glass.** The two free slots are the *exact mirrors*
   of the whiskey and cola as seen from arm B (r = 0.539 and 0.586, same
   arm-frame coordinates), so a glass at the mirror point `(-0.04, 0.55)`
   makes arm B's station geometrically identical to arm A's and the taught
   joint angles transfer unchanged. The work is in `pour_action_server`,
   which hard-codes `base_link` and `ur_manipulator`.
3. **Re-pitch the line.** Arms out to `y = ±0.50`, pitch down to 0.13, seven
   slots. Buys the most positions, but 130 mm against the whiskey's 103 mm
   across-corners is tight, and every taught point needs re-measuring.

**Recommendation: 2.** It doubles the bar's capability rather than its
inventory, and the mirror symmetry means the geometry is already proven.

### Phase E — perception

Replace `layout.py`'s fixed station poses with something that looks. The API
is already shaped for it: every pose carries `source` and `confidence`, so
callers written in Phase B keep working when ground truth becomes a camera.

`layout.py` stays — it becomes the *prior* and the reachability oracle
rather than the source of pose truth.

### Phase F — hardware

The boundary is `ros2_control`. Recalibrate `layout.py`, replace the
hardware interface, and find out which of the sim's conveniences were load-
bearing. `DetachableJoint` for the cap is the obvious one that does not
survive.

## Ideas worth considering

Not committed to. Roughly by value-per-effort.

**A recipe layer.** Above the skills: "negroni" → a sequence of pours. The
pipeline machinery in `bartender_teach` is most of this already; it needs
quantities and a notion of ingredients rather than stations.

**Replay a failed job.** Every job already knows its stage and its
measurements. Storing them and offering `POST /job/{id}/replay` would make
the flaky failures far easier to characterise — right now every
investigation starts by rebuilding the situation by hand.

**A soak harness.** `relaunch.sh` and friends in the scratchpad are ad hoc
and have been rewritten several times this project. A proper `make soak
N=20` that runs a skill N times on fresh stacks and reports a success rate
would turn "the open is flaky" into a number, and would tell you whether a
fix worked.

**Generate `GET /skills` from the `.action` files.** Keeps the agent-facing
schema honest for free, and makes adding a skill a one-place change.

**An MCP front end** once the verb set settles. Maps nearly one-to-one onto
`/skills`.

**Voice or a chat front end.** Cheap once the API exists; the actions
already emit human-readable `state` feedback designed for narration.

**A second glass, and two drinks at once.** Falls out of Phase D option 2.

**Force feedback instead of position-only grasping.** Several of the grip
problems here are really "we can only measure where the fingers are, not
what they are holding". A simulated force sensor would be a research
direction rather than a fix.

## Non-goals, for now

- **Real-time control.** Everything here is plan-then-execute and should
  stay that way.
- **Multi-robot.** Two arms on one controller is enough.
- **A mobile base.** The bar comes to the robot.
