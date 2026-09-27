# Orange cup live tracker

Locates the orange ribbed cup in the `cam0_upside` camera feed, for pour
planning. Not a trained model: DINOv2 (frozen, off-the-shelf) patch features,
scored against a handful of hand-boxed reference crops of the cup and its
background. Ground truth for those crops is
[`photos/Central/labels.jsonl`](../photos/Central/labels.jsonl).

Coordinates are **pixels in the `cam0_upside` image (1280x720)**, not world
or robot coordinates. World-frame pouring needs the D405 depth camera, which
is not wired into this yet (see Known issues).

## Architecture

```text
Pi (10.42.0.200)                 Brev GPU box (training-center-point)
  cameras -> ROS2 topics            cup_center_live.py (A100, DINOv2)
  mjpeg_bridge.py:8765   <--tunnel-->  serves :8767
    /cam0_upside                        /stream       (annotated view)
    /cam1_stand                         /position     (JSON: this doc)
    /cup_tracked   (proxies :8767/stream)
    /cup_position  (proxies :8767/position, + republishes on ROS2)
```

The Pi has no GPU, so tracking runs on Brev and is proxied back onto the
Pi's network. The tunnel between them runs on whichever laptop is doing this
work that day — see **Running it** below. This is a real weak point (see
Known issues): if that laptop's tunnel drops, tracking stops until someone
notices and restarts it.

## Consuming the cup position

**HTTP**, from anywhere on the `10.42.0.x` network:

```text
GET http://10.42.0.200:8765/cup_position
{"found": true, "x": 808, "y": 502, "score": 0.205, "updated_at": 1790473865.83}
```

**ROS 2**, topic `/cup/position` (`std_msgs/String`, same JSON body). Not
yet confirmed working end-to-end — see Known issues.

A consumer must check both `found` and how old `updated_at` is (e.g. reject
anything older than ~1s) before treating `x, y` as real. `found: false` means
the cup is out of view, occluded, or match confidence is below
`--min-score` (default 0.08) -- not a fallback zero position. Do not pour on
a stale or `found: false` reading.

## Running it

Needs the Pi's camera stack already running (`~/pi_code/run.sh`, see
`~/pi_code/README.md` on the Pi) and a Brev instance with a GPU
(`training-center-point` at time of writing).

**1. On Brev**, once per session or after any restart:

```bash
ssh training-center-point
cd ~/cup   # or wherever cup_center_live.py + reference images live
pip install -q "numpy<2" "opencv-python<4.11" "opencv-python-headless<4.11" torch transformers pillow

python3 cup_center_live.py \
  --ref live/live0d_frame.jpg --ref-box 822 410 918 496 \
  --ref live/newref.jpg       --ref-box 950 455 1050 535 \
  --ref live/fresh_frame.jpg  --ref-box 1008 322 1115 408 \
  --ref live/cal1.jpg         --ref-box 855 290 950 378
```

Each `--ref`/`--ref-box` pair is one frame from `cam0_upside` and a tight
box around the cup in it. More frames = better background rejection (see
Known issues) but slower start-up. Re-enrol (restart with fresh crops)
whenever the cup or the camera's view of the table changes meaningfully.

**2. Tunnel** the Pi's camera into Brev and Brev's tracker view back out,
from a machine that can reach the Pi (`10.42.0.x` network):

```bash
ssh -N -R 8765:10.42.0.200:8765 -L 0.0.0.0:8767:localhost:8767 training-center-point
```

Leave this running. If it dies, `/cup_position` starts returning
`{"found": false, "error": "tracker unreachable: ..."}`  -- that's the signal
to restart this command.

**3. On the Pi**, in `~/pi_code/` (not in git yet -- lives only on the Pi's
disk at the time of writing; `scp` a copy from there, or from whoever has
one, if you need it elsewhere). `mjpeg_bridge.py` needs to know that machine's IP as
`CUP_TRACKED_URL` / `CUP_POSITION_URL` (currently hardcoded to
`10.42.0.50`, the laptop used to build this -- change it if running the
tunnel from a different machine), then:

```bash
ssh bartender@10.42.0.200
cd ~/pi_code
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/bartender/pi_code/.fastdds.xml
python3 mjpeg_bridge.py --port 8765
```

**Check it's alive:** open `http://10.42.0.200:8765/` -- three views,
`cup_tracked` should show a red box and green crosshair on the cup, or
"CUP NOT FOUND" in red if it's out of view.

## Known issues / next steps

- **No depth, no world coordinates.** This gives pixel position only. The
  D405 wrist camera is connected but its ROS node was stuck in a USB
  control-error loop (`xioctl(UVCIOC_CTRL_QUERY)` failures) last checked --
  likely missing udev rules. Fixing that is the path to real 3D pour
  targets; `bartender_api/perception.py`'s `fit_centre()` already has the
  matching depth-circle-fit logic for the sim camera and is the template to
  reuse.
- **The tunnel is a manual, single point of failure.** It has died twice in
  one session already. Before relying on this for anything unattended, turn
  it into a systemd service (or similar) with auto-restart on both ends,
  rather than a command someone keeps in a terminal.
- **`/cup/position` ROS2 topic was added but not confirmed receiving
  messages** in the last check (`ros2 topic echo` reported nothing
  published). Verify before depending on it; the HTTP endpoint is confirmed
  working.
- **Reference crops are hand-picked pixel boxes**, tied to `cam0_upside`'s
  current mounting and field of view. If the camera moves, re-shoot and
  re-box.
- **Centre offset is a single hardcoded correction**
  (`CENTRE_OFFSET` in `cup_center_live.py`), measured near the middle of the
  table. It was tuned against 4 reference frames; accuracy is roughly
  10-20px (~1-2cm) and may vary near the table edges, where the camera sees
  the cup from a different angle. If pouring needs tighter accuracy, replace
  this with a proper circle fit on the rim in a small crop around the
  tracked point (cheap once the cup is roughly located, unlike a whole-image
  search).
