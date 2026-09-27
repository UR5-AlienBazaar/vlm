# vlm

`vlm` is the vision-language model inference boundary for the robot fleet. It
owns model runtime, model versions, training and evaluation. It does **not**
own ROS nodes, robot motion, camera drivers, or robot safety decisions.

The model is Qwen3-VL served by vLLM on the GPU instance; this repo talks to
its OpenAI-compatible endpoint. Nothing here runs the model locally.

## Run

On the GPU instance:

```bash
vllm serve <sft-r1 checkpoint> --served-model-name sft-r1 --port 8101
```

From the laptop, tunnel it and test:

```powershell
ssh -L 8101:localhost:8101 shadeform@<host>
python training/eval_bottles.py photos --strict-contract --out bottles_sft_r1_strict.jsonl
python training/stream_vlm.py --source 0 --endpoint http://localhost:8101 --model sft-r1 --show
```

## API

`POST /v1/scene` accepts a JSON request containing one base64-encoded image.
The canonical contract is [`contracts/scene-v1.schema.json`](contracts/scene-v1.schema.json).

```json
{
  "request_id": "scene-42",
  "camera": {"id": "stand", "captured_at": "2026-09-26T12:00:00Z"},
  "image": {"content_type": "image/jpeg", "data_base64": "..."}
}
```

The caller must act only on an `observed` response and must reject stale
timestamps or confidence below its own policy threshold. A VLM response is
perception evidence, never a motion command.

## Integration rule

The robot repository should call this service through HTTP and translate the
result into its ROS world/perception message. Do not import this repository's
Python source into a ROS package and do not let this service call robot
controls. Pin deployment to an image digest or release tag and retain
`model.name` / `model.version` in logs and recorded episodes.

## Development

```powershell
python -m pytest training
```

## Training data

`training/vlm_labels.py` turns Gazebo segmentation-camera captures from
[bartender_robot_sim](https://github.com/UR5-AlienBazaar/bartender_robot_sim)
into a chat-format JSONL dataset. Its `LABEL_IDS` must match the `<label>`
values in that repo's `bar_world.sdf`:

```powershell
$env:BAR_WORLD_SDF = "..\bartender_robot_sim\ros2_ws\src\bartender_gazebo\worlds\bar_world.sdf"
python -m pytest training
```

## Cup position for pour planning

`training/cup_center_live.py` locates the orange cup in the real overhead
camera feed and serves its pixel position for the pour planner to consume.
See [`training/CUP_TRACKER.md`](training/CUP_TRACKER.md) for how to run it,
the HTTP/ROS2 endpoints, and known limitations (pixel-only, no depth yet).
