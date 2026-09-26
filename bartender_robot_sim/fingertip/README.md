# Neck-hooking fingertip for a Robotiq 2F-85

A parametric generator for a custom fingertip that lets a UR5e pick glass
bottles up **by the neck**. The tip hooks under the bottle's collar ring, so
the bottle hangs on a mechanical ledge instead of being held by pad friction.

`fingertip.py` builds a mirrored left/right pair with [build123d] and exports
STEP (for CAD) and STL (for slicing).

[build123d]: https://build123d.readthedocs.io/

---

## Setup

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

`requirements-dev.txt` adds pytest and matplotlib; matplotlib is only used by
`preview.py`, which renders the exported STLs to a PNG so you can glance at the
shape without opening CAD.

## Use

```bash
# the pair, with your measured bottle dimensions
python fingertip.py --neck-d 30 --collar-d 36 --out ./build

# just the mounting plate: a 5-minute test print to check the bolt pattern
python fingertip.py --mount-check --out ./build

# plain through-holes instead of M4 countersinks
python fingertip.py --no-countersink
```

Everything else lives in the `Params` dataclass at the top of `fingertip.py`.
Edit it there once you have measured real bottles; the two CLI flags are just
the dimensions that change most often.

Run the tests with `pytest` (a `pytest.ini` disables the ROS 2 pytest plugins
that this machine puts on `PYTHONPATH`, so no wrapper is needed).

---

## Read this before you print anything that bolts on

**The mounting dimensions in `Params` are invented placeholders.** They are not
Robotiq's, and they are not measured. They exist only so the model has
something to build.

```
mount_hole_spacing  mount_hole_d  mount_face_w  mount_face_h  mount_plate_t
```

Replace all five with values measured off the real fingertip, or read out of
Robotiq's official 2F-85 STEP file. Until you do, every run prints a loud
warning naming the ones still untouched. Then print `--mount-check` and offer
the plate up to the gripper **before** committing to a full tip.

---

## Parameters

### Bottle

You usually should not need to measure these per bottle. The **finish** -- the
neck and mouth -- is the most standardised part of a glass bottle, because
closures have to fit it, and the ring this tip hooks under is normally the
**transfer bead**, which exists so factory conveyors can carry bottles by the
neck. Neck handling is established practice; this is a small version of it.

`--list-finishes` shows the known ones (crown, BVS 30H60, wine cork, GPI 400).
Their dimensions are deliberately **not filled in**: they have to come from the
glass supplier's finish drawing, the GPI/SPI or CETIE standard sheet, or a
caliper. Fill them into `FINISHES` and record `source` alongside. Using an
unfilled finish exits 2 rather than guessing.

| name | meaning |
|---|---|
| `neck_d` | Neck outside diameter, below the collar. The pocket is sized from this. |
| `collar_d` | Collar ring outside diameter. The difference from `neck_d` is the overhang the ledge hooks under, so this is the single most important measurement. |
| `clearance` | Radial gap between neck and pocket wall. |

### Neck pocket
| name | meaning |
|---|---|
| `pocket_arc` | How much of the neck one tip wraps, in degrees. Two tips at 120° give 240° of wrap. |
| `pocket_h` | Pocket height. Defaults to the full body height, so the "pocket floor" is the bottom face of the tip and the bottom lead-in becomes a true entry funnel. Set it shorter to leave a closed floor. |

The pocket is a vertical cylinder centred *ahead* of the front face. To subtend
exactly `pocket_arc`, its axis sits at `front_x + r·cos(arc/2)`, which leaves a
bite of `r·(1 − cos(arc/2))` — 7.8 mm at the defaults.

### Collar ledge — the load-bearing surface
| name | meaning |
|---|---|
| `ledge_h` | Height of the ledge above the pocket floor. Below it the wall hugs the neck and stops the bottle swinging; above it the wall is cut back to clear the collar. Wants to be most of `pocket_h`. |
| `ledge_depth` | How far the wall steps outward above the ledge. |

### Lead-in taper
| name | meaning |
|---|---|
| `leadin_angle` | Taper angle from the closing axis, at both the pocket bottom and the pocket entry. |
| `leadin_h` | Depth of the lead-in, along Z at the bottom and along X at the entry. |

`leadin_angle` **cannot be as shallow as it looks.** The pocket wall already
leaves the entry at `90 − pocket_arc/2` degrees from the closing axis, so at a
120° arc a 30° lead-in is exactly degenerate and cuts nothing. Validation
refuses it and tells you the minimum.

### Body and limits
| name | meaning |
|---|---|
| `body_t`, `body_w`, `body_h` | Tip thickness, width and height. |
| `wall_min` | Thinnest wall allowed behind the pocket. |
| `max_stroke` | The 2F-85's 85 mm maximum opening. |
| `min_opening` | Opening that must still be available with tips fitted. |
| `countersink`, `csk_head_d`, `csk_angle` | M4 countersink, per ISO 10642. |

`body_w` has a floor that is easy to miss: the **collar** pocket is a larger
circle cut by the same plane, so it reaches round further than the neck pocket
— 131° versus 120° at the defaults. Too narrow a body and it breaks out of the
sides and truncates the ledge you are hanging the bottle from. Validation
computes the minimum and refuses.

---

## The ledge check

The brief asked validation to assert **`ledge_depth < collar overhang`**. That
is asserted the other way round here, deliberately, and it is worth a moment
because it changes the part.

For the bottle to hang, its collar has to sit *above* the ledge. So the pocket
above the ledge must be **wider** than the collar, or the collar cannot get
there at all — it fouls the wall on the way in:

```
    pocket_r + ledge_depth  >=  collar_r + clearance
```

If `ledge_depth` were *less* than the overhang, the step would stop short of
the collar's rim and the collar would interfere with the upper wall rather
than rest on the ledge.

The bearing surface does not come from `ledge_depth` at all. The ledge annulus
runs from `pocket_r` to `pocket_r + ledge_depth`; the collar's underside runs
from `neck_r` to `collar_r`. What carries the bottle is the overlap, so once
the step clears the collar the bearing width is just the collar overhang
(2.40 mm at the defaults) and making `ledge_depth` larger does not add to it.

If you meant something different by `LEDGE_DEPTH` — measuring it from the
collar rather than from the neck pocket, say — tell me and I will flip it back.

---

## If you want to try this in the Gazebo sim first

The bottles in this repo will not work with a neck hook as they stand, and the
reason is worth knowing before you print anything.

Both `models/jack_daniels_bottle` and `models/cola_bottle` end in a 33 mm neck
with a 50 mm shoulder **below** it. Going up the bottle the profile only ever
gets narrower, so there is nothing above the neck for a ledge to hook under.
The generator says so rather than building a part that cannot work:

```
$ python fingertip.py --neck-d 33 --collar-d 33
ERROR: fingertip geometry is not buildable:
  - collar_d 33.0 must exceed neck_d 33.0: with no collar overhang there is
    nothing for the ledge to hook under
```

To test the mechanism in simulation, add a collar ring to those models: a
short cylinder at the top of the neck, wider than the neck. A real 750 ml
spirits bottle with a 33 mm neck has a lip around 38-39 mm, so something like

```xml
<collision name="bottle_collar_collision">
  <pose>0 0 0.2400 0 0 0</pose>
  <geometry><cylinder><radius>0.0195</radius><length>0.008</length></cylinder></geometry>
</collision>
```

At `--neck-d 33 --collar-d 39` the generator then wants `body_w` raised to at
least 40.82 mm, and tells you so.

---

## Printing

**Material: TPU 95A.** Grippy enough that the neck does not skid in the pocket,
and tough enough in layer adhesion to carry a full bottle.

**Orientation: mounting face flat on the bed**, so the part grows along its
own X axis toward the bottle. This matters more than it sounds. The ledge is a
horizontal surface and the bottle's weight pulls straight down on it. Printed
standing up, the ledge lands *on* a layer boundary and the load peels the
layers apart in tension — the weakest thing you can ask of an FDM part.
Mounting-face-down turns the layer planes 90°, so the same load is carried in
shear across many layers instead. Expect to need supports in the pocket at
this orientation; that surface is not load-bearing, so a scarred finish there
costs nothing.

**Walls: 4–5 perimeters** (≈2 mm at a 0.4 mm nozzle). In TPU the perimeters do
essentially all the work; infill contributes little. 40–60 % gyroid behind
them is plenty.

Other settings that matter for TPU: 0.2 mm layers, 20–30 mm/s, no or minimal
retraction, and dry filament — TPU absorbs moisture quickly and prints stringy
and weak when wet.

**Test print first.** `--mount-check` gives just the plate, a few minutes of
filament, and will tell you whether the bolt pattern is right before you spend
an hour on a full tip.

---

## Validation

`validate()` runs before every export and **raises rather than clamping** — a
silently adjusted dimension is a part that fits nothing. It checks the pocket
radius is positive, the collar is wider than the neck, the ledge clears the
collar and has a real bearing width, the lead-in is not degenerate, enough
wall is left behind the pocket, the body contains the collar pocket, the tips
still leave `min_opening` of the 85 mm stroke, the bolts fit the mounting face,
and the countersinks do not break through. Messages name the numbers that
failed and what to change.

It warns, rather than failing, when the mounting dimensions are still
placeholders, and when the plate overhangs the body.
