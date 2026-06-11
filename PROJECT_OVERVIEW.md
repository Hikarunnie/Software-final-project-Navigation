# DuckieTown Navigation Platform — Project Overview

## Purpose

A robotics education platform for Duckiebot robots. Students complete programming tasks that run either in a **Godot 4.6 simulation** or on a **real Duckiebot**. Each task combines Jupyter notebooks (theory), Python packages (student-editable code), Flask web servers (live UI), and Godot 3D scenes (simulated world).

## Architecture

```
  Browser (Web UI)
       |
  [HTTP / MJPEG]
       |
  Flask Web Server  (servers/<task>/)
       |
  +---------+---------+
  |                   |
  Student Code        Hardware Drivers
  (tasks/<task>/      (duckiebot/*/)
   packages/)
  |                   |
  +---------+---------+
       |
  Godot Simulator  OR  Real Duckiebot
  (network sockets)     (GPIO/I2C/serial/camera)
```

## Launch Entry Point

**`launch.py`** — CLI entry point with commands:
- `--sim --task <name>` — launch in Godot simulation
- `--run --bot <name> --task <name>` — deploy to real robot
- `--stop --bot <name>` — stop task on robot

Handles Godot auto-download, port discovery, server startup, and packaging/deployment.

---

## Folder: `config/` — Configuration YAML files

Tunable parameters persisted across restarts, editable via web UI:

| File | Purpose |
|------|---------|
| `braitenberg_config.yaml` | Braitenberg agent: base speed, gain, detection threshold |
| `braitenberg_hsv_config.yaml` | HSV color bounds for duckie detection |
| `lane_servoing_config.yaml` | Lane follower PD gains, speeds, curve detection |
| `lane_servoing_hsv_config.yaml` | HSV bounds for yellow (center) and white (right) lines |
| `modcon_config.yaml` | Robot dimensions, PID gains, encoder resolution |
| `object_detection_config.yaml` | YOLO model params (img size, thresholds) |
| `project_config.yaml` | Project task HSV bounds (yellow + white lines) |

---

## Folder: `launcher/` — Launch utilities

| File | Purpose |
|------|---------|
| `__init__.py` | Exports scene map, path constants, port helpers |
| `config.py` | Path definitions + task-to-Godot scene mapping |
| `ports.py` | Free TCP port finder + Godot port-file waiter |

---

## Folder: `godot/` — Godot integration

| File | Purpose |
|------|---------|
| `utils/map.py` | Parses `.tscn` scene files to extract road network graph (nodes = intersections, edges = roads with lengths) for path planning |

---

## Folder: `servers/` — Flask web servers

Each task has a `virtual_server.py` (simulation) and optionally a `real_server.py` (hardware).

### Common modules
| File | Purpose |
|------|---------|
| `common.py` | HTTP log suppression, MJPEG frame generator, cleanup helper |

### Templates (`servers/templates/`)
| File | Purpose |
|------|---------|
| `base.py` | Dark CSS theme + shared JS (status, sliders, JSON posting) + `render_template()` |
| `braitenberg.py` | Motor bars, HSV sliders, agent config, reset button |
| `introduction.py` | Keyboard controls, wheel speeds, LED color pickers |
| `lane_servoing.py` | HSV sliders (yellow+white), start/stop drive, lane PD params |
| `modcon.py` | Pose display, path canvas, PID tuning, wheel calibration |
| `object_detection.py` | Mode toggle, keyboard control, confidence threshold, detection list |
| `project.py` | HSV calibration, manual/nav mode, start/goal selectors, path display, dance button, lane control |

### Servers by task
| Server | Routes | Key behavior |
|--------|--------|-------------|
| **Braitenberg** `virtual_server.py` | `/`, `/video`, `/get_stats`, `/reset_game`, `/get_hsv`, `/update_hsv`, `/get_motors`, `/update_config` | 20Hz control loop, 2×2 visualization (camera, heatmap, weight matrices) |
| **Introduction** `virtual_server.py` | `/`, `/video`, `/keys`, `/speeds`, `/wheels`, `/snapshot`, `/leds` | Keyboard → motor speeds via student code |
| **ModCon** `virtual_server.py` | `/`, `/video`, `/status`, `/maneuver`, `/stop`, `/reset_pose`, `/reset_sim`, `/update_pid`, `/save_calibration` | Odometry + PID + blocking maneuvers in background threads |
| **Lane Servoing** `virtual_server.py` + `visualization.py` | `/`, `/video`, `/reset`, `/update_config`, `/get_hsv`, `/update_hsv`, `/start`, `/stop`, `/running`, `/status` | PD lane follower, 2×2 lane visualization |
| **Object Detection** `virtual_server.py` / `real_server.py` + `visualization.py` | `/`, `/video`, `/start`, `/stop`, `/reset`, `/set_mode`, `/switch_scene`, `/keys`, `/remove_objects`, `/set_threshold`, `/status` | YOLO detection thread + lane following |
| **Project** `virtual_server.py` / `real_server.py` | `/`, `/video`, `/status`, `/keys`, `/set_mode`, `/set_start`, `/set_goal`, `/get_start`, `/get_goal`, `/route`, `/maneuver` | Navigation: lane following + Dijkstra path planning + dance |

---

## Folder: `tasks/` — Student task packages

### Introduction
| File | Purpose |
|------|---------|
| `packages/manual_drive.py` | Student: `get_motor_speeds(keys)` → differential drive speeds |
| `packages/led_control.py` | Student: `set_turning_leds(direction)` → LED colors via colorsys |

### Braitenberg
| File | Purpose |
|------|---------|
| `packages/agent.py` | `BraitenbergAgent` — duckie-avoiding Braitenberg vehicle |
| `packages/preprocessing.py` | HSV mask for duckie detection |
| `packages/connections.py` | Weight matrices (left/right inhibitory) |

### Visual Lane Servoing
| File | Purpose |
|------|---------|
| `packages/visual_servoing_activity.py` | Student: `detect_lane_markings()` — HSV + Canny lane detection |
| `packages/agent.py` | `LaneServoingAgent` — PD controller with curve detection, boundary penalties |
| `packages/cuvrve_behavior.py` | Curve detection via pixel shift across vertical slices |

### ModCon (Motion Control)
| File | Purpose |
|------|---------|
| `packages/odometry_activity.py` | Student: `delta_phi()` + `pose_estimation()` — differential drive odometry |
| `packages/pid_controller.py` | Student: `PIDController()` — standard PID with wrapped angle |
| `packages/agent.py` | `ModConAgent` — straight/turn/square maneuvers |
| `packages/tests/unit_test.py` | Unit tests for odometry and PID |

### Object Detection
| File | Purpose |
|------|---------|
| `packages/agent.py` | `ObjectDetectionAgent` — YOLOv5 ONNX inference with multiple backends |
| `packages/stop_activity.py` | Student: `should_stop(detections)` — stop logic |
| `packages/integration_activity.py` | Student: class/size/score filters |
| `packages/dataset_activity.py` | Student: LabelMe → YOLO annotation converter |
| `packages/prepare_dataset.py` | Dataset preparation: resize, split, format |

### Project (Final Navigation)
| File | Purpose |
|------|---------|
| `packages/agent.py` | `NavigationAgent` — integrates lane following, red line detection, Dijkstra path planning, intersection FSM, dance |
| `packages/optimal_path.py` | `dijkstra()` — priority-queue shortest path on road graph |
| `packages/road_map.py` | `RoadMap` — weighted graph from Godot scene or hardcoded map |

---

## Configuration file `PROJECT_OVERVIEW.md` created
