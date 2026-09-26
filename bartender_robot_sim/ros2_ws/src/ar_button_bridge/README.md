# ar_button_bridge

Publishes the three buttons on a Spectacles AR UI to a ROS 2 topic.

```
Spectacles Lens --HTTPS POST--> ar_button_bridge (this node) --> /ar/button --> robot node
```

The glasses can't be a ROS node, so the Lens sends a small HTTP request and
this node republishes it.

## The contract (for whoever subscribes)

| | |
|---|---|
| Topic | `/ar/button` |
| Type | `std_msgs/msg/Int32` |
| Values | `1`, `2` or `3`, one message per press of that button |
| QoS | default reliable, depth 10 |

A message means "the user pressed this button once". What each number does is
up to the subscriber. The bridge only ever publishes 1, 2 or 3, and drops a
repeat of the same button within 0.5 s (`--min-interval`).

There is no stop or release message yet. If one is needed, agree on another
value (for example `0`) and it is a one-line change in `bridge.py`
(`VALID_BUTTONS`) and in the Lens.

## Run it

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --packages-select ar_button_bridge
source install/setup.bash

ros2 run ar_button_bridge ar_button_bridge --token SOMETHING     # the bridge
ros2 run ar_button_bridge ar_button_listener                     # stands in for the robot
```

Options: `--host` (default `0.0.0.0`), `--port` (default `8765`), `--token`,
`--min-interval`, `--topic`, and `--dry-run` (no ROS at all: it only prints the
buttons it accepts, so the glasses can be tested from a computer without ROS).

The bridge listens on every interface so the glasses can reach it. There is no
encryption and, without `--token`, no authentication: **anyone who can reach
the port can publish a button.** Use a network you trust and set `--token`
(the Lens sends it in the `X-Token` header).

Check it without the glasses:

```bash
curl http://localhost:8765/health
curl -X POST http://localhost:8765/button -H 'Content-Type: application/json' \
     -H 'X-Token: SOMETHING' -d '{"button": 2}'
ros2 topic echo /ar/button
```

## The Lens side

Spectacles needs **HTTPS** for `fetch` (see `ar/drinkButtons/README.md` in
this repo, "Robot link"), so plain `http://<lan-ip>:8765` will not work from the
glasses. Put the bridge behind an HTTPS front, such as a tunnel or Tailscale,
and give the Lens that address. The bridge itself stays plain HTTP behind it.

In the Lens, set **Bridge Url** (and **Bridge Token**) on the script, for
example `https://<your-tunnel>`. Pressing a button sends `{"button": N}` to
`<Bridge Url>/button`. Turning on Experimental API in Lens Studio's Project
Settings may also be needed for internet access. Neither has been tested on the
glasses yet.

## Tests

The request handling has no ROS in it, so it runs anywhere with Python:

```bash
python3 -m unittest discover -s test
```

Under `colcon test` the flake8 and pep257 checks run too, like every other
package here.
