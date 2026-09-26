# Workcell API: picking bottles and making drinks

An HTTP/JSON API for the one-arm UR5e workcell. Another program (an
ordering script, a web page, a voice assistant) calls it to pick a bottle or
make a drink. Every call runs scripts that were taught on the teach pendant.
The API can't send the arm anywhere that wasn't taught.

For setting up the robot itself, see [WORKCELL.md](WORKCELL.md).

## Quick start

```bash
# 1. The robot stack (real robot + Gazebo twin), in one terminal:
ros2 launch bartender_bringup workcell_twin.launch.py

# 2. The API server, in another:
ros2 run bartender_api server --points workcell
```

```bash
curl http://127.0.0.1:8090/bottles
curl -X POST http://127.0.0.1:8090/pick \
  -H 'Content-Type: application/json' -d '{"bottle": "whiskey"}'
```

The server prints its address, the point file and the menu file it uses.

## Access

| | |
|---|---|
| Address | `http://127.0.0.1:8090` (default: this computer only) |
| Port | `--port 8090` |
| From another computer | `--host 0.0.0.0` (all interfaces), or `--host <this PC's LAN address>` |
| Authentication | **None** |
| Format | JSON in, JSON out. POST bodies need `Content-Type: application/json` |

> **Anyone who can reach the port can move the robot.** There is no
> password or token. Keep the default (`127.0.0.1`) unless the calling
> program is on another computer, and then bind only on a network you trust,
> such as the robot's own cable or a VPN. Never forward the port to the
> internet.

### Server options

| Option | Default | Meaning |
|---|---|---|
| `--points NAME` | the two-arm bar's `taught_points.yaml` | Point file. `workcell` means `bartender_teach/config/workcell_points.yaml`. A path also works. |
| `--menu NAME` | the `--points` name | Drinks menu. With `--points workcell` it is `bartender_teach/config/workcell_menu.yaml`. |
| `--host`, `--port` | `127.0.0.1`, `8090` | Where to listen. |

The server reads the point file and the menu again on every request. A
bottle or drink taught while the server runs is available on the next call,
without a restart.

## Endpoints

| Method | Path | Body | Does |
|---|---|---|---|
| GET | `/bottles` | | List the bottles that can be picked |
| POST | `/pick` | `{"bottle": "whiskey"}` | Pick one up (run `grab_<bottle>`) |
| GET | `/drinks` | | List the menu, and which drinks are ready |
| POST | `/make` | `{"drink": "whiskey_cola"}` | Make one drink (run all its scripts) |
| GET | `/state` | | Joint angles, tool position, gripper |
| POST | `/move/point` | `{"arm": "a", "point": "home"}` | Go to one taught point |
| POST | `/move/jog` | `{"arm": "a", "axis": "z", "amount": 20}` | Small relative move (mm or degrees) |
| POST | `/gripper` | `{"arm": "a", "position": 0.5}` | Move the gripper |

Names are not case-sensitive (`"Whiskey"` works). The workcell's arm is
always `"a"`.

### GET /bottles

```json
{
  "bottles": [
    {"bottle": "gin",     "pipeline": "grab_gin",     "steps": 6},
    {"bottle": "vodka",   "pipeline": "grab_vodka",   "steps": 6},
    {"bottle": "whiskey", "pipeline": "grab_whiskey", "steps": 6}
  ],
  "points_file": ".../bartender_teach/config/workcell_points.yaml"
}
```

A bottle is on this list when the point file has a pendant script called
`grab_<bottle>`. **To add a bottle, teach `grab_<bottle>` on the pendant.**
No code changes.

### POST /pick

```json
{"bottle": "whiskey"}
```

Runs `grab_whiskey` from start to finish and answers when the arm has
stopped. That takes seconds, or longer at a low speed setting, so allow at
least 60 s before your client times out.

```json
{"ok": true, "bottle": "whiskey",
 "message": "running grab_whiskey: 6 step(s)\n  1/6  goto aproach_grab_whiskey ...\n ... finished grab_whiskey"}
```

### GET /drinks

```json
{
  "drinks": [
    {"drink": "whiskey_cola", "name": "Whiskey & Cola",
     "scripts": ["grab_whiskey", "pour_whiskey", "return_whiskey",
                 "grab_cola", "pour_cola", "return_cola"],
     "ready": false,
     "missing_scripts": ["pour_whiskey", "return_whiskey", "grab_cola",
                         "pour_cola", "return_cola"]}
  ],
  "menu_file": ".../bartender_teach/config/workcell_menu.yaml"
}
```

- `ready` is true when every script, and every point those scripts use, has
  been taught. Only ready drinks can be made.
- `missing_scripts` and `missing_points` list what is still to teach.
- Use `name` for display and `drink` for the request.

The menu has seven drinks: `whiskey_cola`, `whiskey_sprite`, `gin_sprite`,
`gin_fanta`, `vodka_cola`, `vodka_sprite` and `vodka_fanta`. Each runs
`grab_`, `pour_` and `return_` for its spirit, then the same three for its
mixer. To change a drink or add one, edit
[`workcell_menu.yaml`](../ros2_ws/src/bartender_teach/config/workcell_menu.yaml).

### POST /make

```json
{"drink": "whiskey_cola"}
```

Runs the drink's scripts one after another, as a single command. It answers
when the last script has finished, or when one of them stops. Allow several
minutes before your client times out.

```json
{"ok": true, "drink": "whiskey_cola", "message": "..."}
```

When a script stops partway, the robot does not carry on, and the answer
says where it stopped:

```json
{"ok": false, "drink": "whiskey_cola",
 "stopped_at": "script 4 of 6: grab_cola",
 "message": "... STOPPED at step 3 of 6: error_code -6"}
```

**A stopped drink leaves the arm where it stopped, possibly holding a
bottle.** Nothing puts the bottle back automatically. Someone has to look at
the robot before the next order.

## Results and errors

Every POST answers with the same shape:

| Field | Always | Meaning |
|---|---|---|
| `ok` | yes | `true` only if everything ran to the end |
| `message` | yes | What happened, in words: the pendant's own output, or why the request was refused |
| `bottle` / `drink` | when the request got that far | The name that ran |
| `stopped_at` | `/make`, on failure | Which script stopped |

| HTTP status | Meaning |
|---|---|
| `200` | Done. `ok` is `true`. |
| `409` | Refused or failed. `ok` is `false`, and `message` says why. |
| `400` | The body isn't a JSON object (on `/move/*` and `/gripper`, also a missing field). |
| `404` | Unknown path. |

Common refusals. None of these move the arm:

| `message` starts with | Cause | What to do |
|---|---|---|
| `busy: a command is already running` | Another pick or drink is in progress. Requests are refused, not queued. | Wait for the first call to answer, then retry. |
| `no bottle 'rum' to pick` | No `grab_rum` script. The message lists the known bottles. | Use `GET /bottles`, or teach it. |
| `no drink 'mojito' on the menu` | Not in the menu file. | Use `GET /drinks`. |
| `Whiskey & Cola is not ready -- scripts not taught yet: ...` | Scripts or points are missing. | Teach them on the pendant. |
| `pick needs a bottle name` / `make needs a drink name` | The body has no `bottle` / `drink`. | Send it. |

Failures while moving, where the arm had already started:

| In `message` | Usual cause |
|---|---|
| `the control program is not running on the robot` | Checked before the first move, so in this case nothing moved. The robot program has stopped, for example after an e-stop or a protective stop. Run `robot resend` on the teach pendant. See [WORKCELL.md](WORKCELL.md) "Stopping, and recovering". |
| `error_code -4` | The controller gave up on the move (CONTROL_FAILED): a protective stop partway, the robot program stopping, or the arm being blocked. |
| `error_code -6` | The move timed out (TIMED_OUT). The launch files already prevent the usual cause, a low speed slider, so if you see it, rebuild and relaunch the robot stack. |
| `plan ... failed` | MoveIt could not find a path to the point. Is something in the way? |

## Calling it from a program

Python, using the standard library only:

```python
import json
import urllib.request

API = 'http://127.0.0.1:8090'


def call(path, body=None, timeout=600):
    """GET when body is None, otherwise POST it. Returns the JSON answer."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(API + path, data=data,
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.load(resp)
    except urllib.error.HTTPError as err:     # 409 and 400 still carry JSON
        return json.load(err)


ready = [d['drink'] for d in call('/drinks')['drinks'] if d['ready']]
print('can make:', ready)

result = call('/make', {'drink': 'whiskey_cola'})
if result['ok']:
    print('done')
else:
    print('failed:', result.get('stopped_at', ''), result['message'])
```

Tips:

- **Send one command at a time**, and wait for the answer before sending
  the next. A second request while the arm moves gets `busy`.
- **Check `GET /drinks` before offering a drink**, and offer only the ones
  that are `ready`.
- **Treat `ok: false` after motion as "stop and get a person"**, not "retry".
  The arm may be holding a bottle.
- **The speed is the UR speed slider**, set with `speed 20` on the teach
  pendant. The API does not change it.
- **The teach pendant is a separate program.** Its moves are not blocked
  while the API is moving the arm, so don't use both at once.

## Adding things

| To add | Do this | Code change |
|---|---|---|
| A bottle to `/pick` | Teach `grab_<bottle>` on the pendant (`record grab_<bottle>`) | none |
| A pour or return step | Teach `pour_<x>` / `return_<x>` the same way | none |
| A drink | Add it to `workcell_menu.yaml` with its list of scripts | none |

How to teach a script is in [WORKCELL.md](WORKCELL.md), "Teaching the
bottles". The server is in
[`bartender_api/server.py`](../ros2_ws/src/bartender_api/bartender_api/server.py),
and picking and making drinks are in
[`movement.py`](../ros2_ws/src/bartender_api/bartender_api/movement.py).
