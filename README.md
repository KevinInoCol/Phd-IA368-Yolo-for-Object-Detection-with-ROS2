# YOLO-Based Potential Field Navigation

`myRobot` collects bananas and avoids poops in CoppeliaSim using
**Kinect RGB-D → YOLO → 3D position → persistent object map → potential field → differential drive**,
all in ROS2 (Jazzy). Object positions come only from the camera: the `banana`/`poop` signals are never read.

## Contents

| Path | What it is |
|---|---|
| `ros2_ws/src/ia368_pkg/` | Course package (unchanged). Uses its `kinect_node` and `tf_node`. |
| `ros2_ws/src/yolo_pf/` | The solution package (4 nodes + launch file + trained model). |
| `ros2_ws/src/yolo_pf/models/banana_poop.pt` | YOLO11s trained on 2 classes: `banana`, `poop`. |
| `training/generate_dataset.py` | Builds the training set from the simulation with automatic labels. |
| `obstacle avoidance pega banana/` | Course scene (version from the course repo). |

## Why a custom YOLO model

The pretrained COCO models (`yolo11n-seg`, `yolo11x-seg`) and YOLO-World with text prompts
("poop", "donut", "brown pile") all detect the bananas, but **none detects the poops** of this scene.
So the robot's detector is a YOLO11s fine-tuned on images rendered from the scene itself:

* the robot is placed at random poses on random maps, and the Kinect RGB image is captured exactly as `kinect_node` publishes it;
* every visible banana/poop is labelled by projecting its mesh vertices with the calibrated camera model (`u = W/2 − f·x/z`, `v = H/2 − f·y/z`, `f = 294.68 px`);
* 560 training images and 44 validation images (≈ 2 800 boxes) were used, with 80 epochs at 320 px.

Validation: banana mAP50 = 0.98, poop mAP50 = 0.95 (precision 0.93 / 0.96, recall 0.97 / 0.95).

## Nodes

1. **`kinect_node`** (course): publishes `/rgb/image` and `/depth/image` and starts the simulation.
2. **`tf_node`** (course): publishes `map → base_link → camera_color_optical_frame`.
3. **`detector_node`**: runs YOLO on each RGB frame. The distance to each object is a low percentile of the depth inside its box, which ignores floor pixels behind it. Depth is converted with the real clipping planes (0.01–3.5 m). The 3D point is computed in the camera frame. Detections farther than 2.5 m or cut by the image border are discarded. Each detection is published as a `Marker` on `/yolo/object_3d_point`.
4. **`object_map_node`**: maintains the **persistent map**.
   * Each detection is transformed to `map` with TF.
   * Re-detections closer than the merge radius (banana 0.30 m, poop 0.45 m) refine the existing object with a running mean instead of inserting it again.
   * An object must be seen 3 times before it is used.
   * Duplicates are consolidated periodically.
   * A banana and a poop at the same spot (< 0.25 m) are the same object with two labels, so the label seen more often wins.
   * Objects stay in the map when they leave the field of view.
5. **`potential_field_node`**: the controller.
   * Attraction: `U_att = ½·k·d²`, conic beyond 1 m, toward the nearest known banana.
   * Repulsion from every mapped poop: `U_rep = ½·k·(1/ρ − 1/ρ₀)²` with ρ₀ = 0.8 m. It has a small tangential (vortex) term against local minima, and it fades near the goal so bananas close to a poop stay reachable.
   * The direction of the resultant force gives the heading. Then `v = min(v_max, |F|)·max(0, cos e)` and `ω = k·e`.
   * A banana closer than 0.15 m is marked collected, and the next one is selected.
   * With no known banana, the robot turns 360° to look around. If it still sees none, it explores toward a random point, still under the field so it avoids poops.
   * A target that the robot stops approaching for 20 s is skipped for 60 s.
6. **`velocity_node`**: converts `cmd_vel` into wheel velocities with `myRobot`'s geometry (r = 0.05 m, L = 0.20 m; a negative joint velocity is forward). The course `vel_node` has the P3DX values.

## Results

Evaluated against the simulator's ground truth (only for measuring, the robot never reads it),
each run on a new random map, with the final version of the nodes:

| Run | Bananas collected | Poops hit | Simulated time |
|---|---|---|---|
| 1 | 20 / 20 | 0 | 102 s |
| 2 | 19 / 20 (run cut at the time limit) | 0 | ~100 s |
| 3 | 20 / 20 | 0 | 81 s |

Persistent map accuracy: median error 6–12 cm for the obstacles, and 16–18 of the 20 real poops
mapped; the others were never in the camera's view.

## Run

Requirements: ROS2 Jazzy, CoppeliaSim 4.10 with the ZMQ remote API on port 23000, and a Python environment with
`ultralytics`, `coppeliasim-zmqremoteapi-client` and `scipy` that can see the ROS packages (venv with `--system-site-packages`).

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
python -m colcon build --symlink-install   # with the venv python, so the nodes run with it
source install/setup.bash
# open "obstacle avoidance pega banana/pega_banana_potential_field.ttt" in CoppeliaSim (do not press play)
ros2 launch yolo_pf yolo_pf.launch.py
```

Stop with **Ctrl+C**: SIGINT closes all nodes.

RViz opens automatically (`rviz:=false` to skip it) in a top view of the `map` frame. It shows the robot and camera TF,
the persistent map (yellow = bananas, brown = poops), the live 3D detections and the annotated YOLO image.
To see which node publishes and subscribes to each topic: `ros2 run rqt_graph rqt_graph`.

> CoppeliaSim Edu periodically opens a modal "Registration" window that **freezes the simulation and the
> ZMQ API** while it is open. Close it, or register the free Edu licence, or the robot will appear stuck.

To regenerate the dataset and retrain:
```bash
python training/generate_dataset.py --port 23000 --maps 14 --poses 70 --out training/dataset
yolo detect train model=yolo11s.pt data=training/dataset/data.yaml imgsz=320 epochs=80 batch=32
```

## Credits

`ros2_ws/src/ia368_pkg` and the CoppeliaSim scene are the course material of IA368 (UNICAMP), from
[cesarbds/IA368ii](https://github.com/cesarbds/IA368ii). The scene's `.ttt` files are not included here.
The `yolo_pf` package, the dataset generator and the trained model are this project's work.
