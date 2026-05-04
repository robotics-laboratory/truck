# lidar_camera_calib

Automatic extrinsic calibration of 2D LiDAR (RPLidar S2) and camera (RealSense D455) for the truck project.

## How it works

Since the RPLidar S2 is a **2D LiDAR** (single horizontal scan plane), traditional 3D LiDAR-camera calibration tools don't apply. This package uses **depth-based alignment**:

1. **Collects** synchronized `LaserScan` + depth image pairs
2. **Converts** LaserScan to 3D points (z=0 plane in lidar frame)
3. **Projects** lidar points onto the depth image using a candidate extrinsic transform
4. **Compares** projected lidar depth vs camera measured depth
5. **Optimizes** the extrinsic (tx, ty, tz, yaw) to minimize depth disagreement

### Degrees of freedom

The node optimizes **4 DOF** for the `base → camera_link` transform:
- `tx`, `ty`, `tz` — translation
- `yaw` — rotation around Z axis

Roll and pitch are assumed zero (rigid flat platform).

## Prerequisites

- Both **LiDAR** and **camera** must be connected and publishing data
- Camera **depth** stream must be enabled (`enable_depth: True`)
- Place the robot facing a **structured scene** with objects at various distances (0.5m–5m). Avoid flat empty walls.

## Build

Inside the Docker container:
```bash
cd /truck
source /opt/ros/iron/setup.bash
make build-all
# or build only this package:
# colcon build --base-paths packages --symlink-install --packages-select lidar_camera_calib
```

## Usage

### Option 1: Standalone launch (recommended for calibration)

This launches camera + lidar + calibration node together:
```bash
source /truck/install/setup.bash
ros2 launch lidar_camera_calib calibration.yaml
```

### Option 2: Run calibration node separately

If truck.yaml is already running (with depth enabled in camera.yaml):
```bash
source /truck/install/setup.bash
ros2 run lidar_camera_calib calibration_node
```

### What happens

1. The node waits for camera intrinsics from `/camera/depth/camera_info`
2. Collects 30 synchronized scan+depth pairs (takes ~10-15 seconds)
3. Runs optimization (Nelder-Mead + L-BFGS-B, takes ~10-30 seconds)
4. Prints the optimal `base → camera_link` transform

### Example output

```
============================================================
  CALIBRATION RESULT (best: L-BFGS-B)
============================================================
  Initial cost:  0.0832 m
  Final cost:    0.0214 m
  Improvement:   0.0618 m

  Translation:
    tx = 0.24312 m
    ty = 0.04876 m
    tz = -0.03521 m

  Rotation:
    yaw = 0.01234 rad (0.71 deg)

  Quaternion (x, y, z, w):
    x = 0.000000
    y = 0.000000
    z = 0.006170
    w = 0.999981

============================================================
  Copy this to model.yaml (tf_static, base -> camera_link):
============================================================

  - frame_id: "base"
    child_frame_id: "camera_link"
    translation: { x: 0.24312, y: 0.04876, z: -0.03521 }
    rotation: { x: 0.000000, y: 0.000000, z: 0.006170, w: 0.999981 }

============================================================
```

## Applying the result

Copy the printed YAML block into `packages/model/config/model.yaml`, replacing the existing `base → camera_link` entry (around line 41-44).

Then restart `truck.yaml` for the new transform to take effect.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_scan_topic` | `/lidar/scan` | LaserScan topic |
| `depth_image_topic` | `/camera/depth/image_rect_raw` | Depth image topic |
| `camera_info_topic` | `/camera/depth/camera_info` | Camera intrinsics topic |
| `init_tx` | `0.25` | Initial X translation (m) |
| `init_ty` | `0.05` | Initial Y translation (m) |
| `init_tz` | `-0.04` | Initial Z translation (m) |
| `init_yaw` | `0.0` | Initial yaw rotation (rad) |
| `num_samples` | `30` | Number of scan+depth pairs to collect |
| `depth_range_min` | `0.3` | Min depth to consider (m) |
| `depth_range_max` | `6.0` | Max depth to consider (m) |
| `depth_scale` | `0.001` | RealSense depth unit (mm→m) |
| `max_depth_error` | `0.3` | Outlier rejection threshold (m) |

## Tips for best results

1. **Scene**: Place 3-5 objects at different distances (boxes, chairs, walls at angles). More structure = better calibration.
2. **Avoid**: Empty rooms, transparent/reflective surfaces, very close objects (<30cm).
3. **Run multiple times**: Results should be consistent (±1cm, ±0.5°). If not, improve the scene.
4. **Increase samples**: Set `num_samples` to 50-100 for more averaging.
