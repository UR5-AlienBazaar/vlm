# Contributing

Written for someone about to make their first change here. The house rules
below are not style preferences; each one is here because breaking it cost
this project a debugging session.

## Get it running

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build
source install/setup.bash
ros2 launch bartender_bringup bartender_sim.launch.py            # with GUI
ros2 launch bartender_bringup bartender_sim.launch.py headless:=true
```

Wait for `You can start planning now` before sending a goal.

```bash
ros2 action send_goal /pour_drink bartender_pour_interfaces/action/PourDrink \
  "{bottle_id: jack_daniels_bottle, glass_id: serving_glass, pour_amount_ml: 45.0}"

ros2 action send_goal /open_bottle bartender_pour_interfaces/action/OpenBottle \
  "{bottle_id: 'beer', stow_after: true}"
```

## Tests

```bash
cd ros2_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test && colcon test-result
```

**`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required, on the BUILD as well as
the test.** Without it you get `ModuleNotFoundError: No module named
'_pytest.scope'`, a clash between the system pytest and a user-installed
`anyio` plugin that has nothing to do with this code.

Setting it only on `colcon test` is not enough, and the way it fails is
nasty. `ament_cmake_pytest` probes pytest at CMake **configure** time; that
probe hits the same clash, and `ament_add_pytest_test` then registers
**nothing, silently**. `bartender_description`'s `render` and
`pep257_repo_style` tests had never run once because of it — the package
reported "3 tests" and they were all linters. Build with the variable set
and it goes from 3 to 6 suites.

If a test count drops after you touch a CMakeLists, check
`build/<pkg>/CTestTestfile.cmake` for the test you expect before assuming
your change is fine.

1091 tests, none of which need a robot. They run in about 13 seconds, so
there is no excuse for not running them.

The `fingertip/` generator is standalone with its own venv:

```bash
cd fingertip && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

## House rules

### Every constant carries the measurement that set it

This is the most important rule in the repo and the reason its files are
comment-heavy. A bare number cannot be re-derived, and someone *will* tune
it later without knowing what it was traded against.

Bad:

```python
BEER_GRASP_HEIGHT = 0.189
```

What the repo actually does: record where the number came from, what was
tried, and what happens if you move it — in this case that 0.191 was tried,
that its two clean runs both failed with a measurably deeper grasp, and that
two runs is not a result but *is* a reason not to assume the band is
interchangeable with itself 2 mm up.

If you change a tuned constant, update its note with what you measured.

### Refuse, don't clamp

`jog z 500` when you meant `50` is a typo. Clamping turns it into a move
that quietly does something other than what was asked, and catching the typo
is the entire reason the bound exists. Same for gripper angles, wait times
and step counts.

### Tests must bite

A test that passes when the code is broken is worse than no test: it buys
false confidence. Two real examples from this repo:

- A regex that matched only list literals silently stopped checking `b_home`
  the moment it became `ARM_B_HOME = list(ARM_A_HOME)`. The test passed by
  checking nothing.
- A guard for "the gripper never moved" sat below a width test that such a
  gripper always takes the other branch of. It never ran once.

**Before you commit a test, break the code and watch it fail.** If it does
not, the test is not testing what you think.

### Report evidence, not booleans

A skill returns the quantity that proves it did its job — how far the cap
moved, how far the bottle shifted — not the fact that its state machine
reached the last state. See `docs/ARCHITECTURE.md`.

### Geometry lives in one place

`bartender_open/layout.py`. If you move something, move it there and let
`test_layout.py` tell you which of the copies (world SDF, URDF, launch file,
`pour_action_server`) needs to follow.

### Publish your own obstacles

A skill must put what it cares about into the planning scene itself, not
assume another server has. `bartender_open` relied on `bartender_pour`
having published the counter, which never happens for a bare open goal —
arm B then planned a path with its forearm 236 mm below the worktop.

### Generated files are generated

`models/whiskey_stand`, `cola_stand`, `beer_stand`, `beer_bottle`,
`beer_cap`, `bottle_opener`, `opener_holster` and the gripper's fingertip
pads all come from scripts. Edit the script and re-run it. A hand edit to
the SDF will be silently overwritten and the reasoning in the generator will
no longer match what is in the world.

### XML comments cannot contain `--`

Both `bar_world.sdf` and `bartender.urdf.xacro` have been broken by this.
Use single hyphens in prose and `=` for dividers. Validate with
`xmllint --noout`.

## Style

- flake8, **max line length 99**
- ament pep257: first line capitalised, imperative, ends with a period,
  blank line after the summary
- Both run as part of `colcon test`, so a lint failure is a test failure

## Working with the simulator

**Kill everything between runs, and verify it died.** Two `move_group`
processes on one bus produce `more than one action server for the action
'move_action'`, goals get preempted mid-flight, and the run is garbage — it
took a wasted investigation to notice. Check for the warning in your log
before believing a result.

**Beware `pkill -f` patterns matching your own shell.** A pattern like
`pkill -f 'ign gazebo'` will match the shell whose command line contains
that text — including the script defining it. Bracket the first character:
`pkill -f '[i]gn gazebo'`. This has killed its own caller twice here.

**Never leave a joint parked on its limit.** The gripper knuckle is
`limit=[0.0, 0.8]`. Drive it to 0.0 — the obvious way to write "open" — and
leave it there, and it stops responding to commands for the rest of the run:
goals still accepted, controller still `active`, joint never moves again.
Measured, 0.000 was dead after a 10 s dwell while 0.020 survived 300 s; the
upper limit is harmless. That is why `GRIPPER_OPEN_POS` is 0.02 and not 0,
in three packages, and why commands outside the band are refused.

If you add an actuator, give its resting positions a margin off both stops
and dwell-test it before believing it works.

**One open per simulator run.** Gazebo's `DetachableJoint` cannot re-attach,
so once the cap is off you must restart to try again.

**The sim runs at roughly 0.2× real time** on an 8-core box (measured off
`/clock`, steady at 0.21 over a two-minute window), so a pour takes
a few minutes of wall clock for ~25 s of simulated time.

## Before you send a change

- [ ] `colcon test` passes, with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` on the
      **build** as well as the test, and the total has not gone *down*
- [ ] New tests fail when you break the thing they test
- [ ] Tuned constants carry a note saying what was measured
- [ ] Geometry changed in `layout.py`, not in a copy
- [ ] If it moves the robot: run it in sim and quote the numbers, and say
      how many attempts it took. This system is intermittently flaky and a
      single green run is weak evidence — say "2 of 3", not "works".
