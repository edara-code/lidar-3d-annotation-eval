# LiDAR 3D Annotation & Evaluation for Autonomous Driving and Robotics

Hands-on annotation of LiDAR point clouds from driving scenes, the kind used by autonomous vehicles and mobile robots, with 3D cuboids, evaluated against official ground-truth labels using 3D IoU, center error, size error, and heading (yaw) error.

## LiDAR in autonomous driving

LiDAR (Light Detection and Ranging) measures distance by sending out laser pulses and timing their return. A spinning sensor on a vehicle's roof collects hundreds of thousands of points per second, building a 3D point cloud of everything around it: cars, people, cyclists, buildings, and the road surface.

### Where LiDAR fits in a self-driving system

| Function | How LiDAR is used |
|---|---|
| **Detection** | Finds and classifies objects in 3D, with exact position and size |
| **Tracking** | Follows each object from frame to frame |
| **Prediction** | Heading and speed are used to predict where objects will move next |
| **Planning** | Accurate distances let the vehicle choose a safe path and speed |
| **Localization & HD maps** | Matches live scans to prebuilt 3D maps to locate the vehicle |
| **Safety validation** | Labeled LiDAR data is used to test perception systems before release |

### Who uses it

| Area | Examples |
|---|---|
| **Robotaxis** | Fully driverless ride-hailing in city traffic |
| **Autonomous trucking** | Long-distance highway freight |
| **ADAS** | Driver-assistance features in consumer vehicles |
| **Delivery robots & shuttles** | Sidewalk delivery, campus and airport shuttles |
| **Mobile & industrial robots** | Warehouse robots, forklifts, and robots navigating around people |

### Why annotation quality matters

Perception models learn from labeled boxes. If boxes are too large, shifted, or turned the wrong way, the model learns those mistakes: it may misjudge how far away a car is, or which way it is heading. Ground-truth labels are also used to test models, so labeling errors can hide real model errors.

## Annotation challenges in driving scenes

| Challenge | Impact on annotation |
|---|---|
| **Sparse points at distance** | Far objects have only a few points, so size and position must be estimated |
| **One-sided view** | The sensor sees only the sides facing it; hidden sides must be inferred |
| **Occlusion** | Parked cars and other objects block parts of each other |
| **Heading** | Front and back can look alike in points; the camera image helps |
| **Ground contact** | The box bottom must sit on the road surface, not above or below it |
| **Similar classes** | Car vs. van, and cyclist vs. pedestrian, need clear rules |

## What this project does

1. **Data:** a KITTI raw drive recorded in Karlsruhe, Germany, with a Velodyne HDL-64E LiDAR and synchronized camera images, taken from Supervisely's public demo project. 10 frames were used: every 10th frame of a 108-frame drive.
2. **Guidelines:** written rules for labeling, in [`guidelines/lidar_3d.md`](guidelines/lidar_3d.md): classes (Car, Van, Truck, Pedestrian, Cyclist), box fitting, heading, ground contact, occlusion, and quality checks.
3. **3D cuboid annotation:** every qualifying object in the 10 frames was labeled in **CVAT** using top, side, and front views plus the camera image, then exported in Supervisely point cloud format.
4. **Evaluation:** a Python script compares my boxes with the official KITTI boxes:

| Metric | What it measures |
|---|---|
| **3D IoU** | Overlap between my box and the official box in 3D |
| **Bird's-eye-view IoU** | Overlap of the box footprints seen from above |
| **Recall** | Share of official objects I labeled, overall and at IoU 0.5 and 0.7 |
| **Center error** | Distance between box centers |
| **Bottom offset** | How closely the box bottom matches the official ground placement |
| **Size error** | Difference in length, width, and height |
| **Heading error** | Angle difference of the box's long side |

Results are broken down by distance from the sensor (0–20 m, 20–40 m, 40 m+), and boxes are counted by class.

## Data and license

The point clouds and images come from the [KITTI dataset](https://www.cvlibs.net/datasets/kitti/) (Geiger et al., 2013), licensed under [CC BY-NC-SA 3.0](https://creativecommons.org/licenses/by-nc-sa/3.0/). The 10 annotated frames are in `data/sample/`.

## Tools

CVAT · Python · pytest

## Results

**Annotated:** 10 frames (0, 10, 20 … 90) of a 108-frame KITTI drive, 77 cuboids in total: 51 Car, 13 Van, 13 Cyclist.

**Ground truth used:** the official labels for this drive cover 3 cars and a tram. In the annotated frames this gives 11 car boxes, all in frames 0, 10, 20 and 30. Accuracy is measured against these 11 boxes, so it is a small sample. The other boxes (parked cars and cyclists) have no official label to compare with, so precision is not reported. KITTI's Car class includes vans, so my Van boxes are compared against it.

**How boxes are compared:** each official box is paired with at most one of my boxes, highest overlap first (bird's-eye-view IoU of at least 0.1). A paired official box counts as found.

### Detection

| Metric | Value |
|---|---|
| Official vehicles found | 10 of 11 (90.9%) |
| Found with 3D IoU ≥ 0.5 | 10 of 11 (90.9%) |
| Found with 3D IoU ≥ 0.7 (KITTI car standard) | 6 of 11 (54.5%) |
| Found with bird's-eye-view IoU ≥ 0.7 | 10 of 11 (90.9%) |

### Box accuracy (10 found boxes)

| Metric | Mean | Median |
|---|---|---|
| 3D IoU | 0.722 | 0.744 |
| Bird's-eye-view IoU | 0.818 | 0.821 |
| Center error, seen from above | 0.16 m | 0.15 m |
| Bottom offset (ground placement) | 0.08 m | 0.06 m |
| Length error | 0.19 m | 0.14 m |
| Width error | 0.15 m | 0.17 m |
| Height error | 0.22 m | 0.11 m |
| Heading error (angle of the long side) | 2.0° | 2.0° |

### By distance from the sensor

| Distance | Official vehicles | Found | Mean 3D IoU | Mean center error (3D) |
|---|---|---|---|---|
| 0–20 m | 4 | 4 | 0.819 | 0.10 m |
| 20–40 m | 5 | 5 | 0.689 | 0.23 m |
| 40 m+ | 2 | 1 | 0.503 | 0.43 m |

### Observations

- **Footprint, position and heading are accurate.** All 10 found boxes pass IoU 0.7 in bird's-eye view, with a median center error of 0.15 m and a heading error of about 2°.
- **Ground placement is accurate.** The median bottom offset is 0.06 m.
- **Box height on distant cars is the main source of error.** The 4 found boxes below 3D IoU 0.7 are all 25 m or farther from the sensor, and they have the 4 largest height errors (0.18–0.77 m). The 6 boxes that pass are all within 31 m.
- **Distant objects are harder.** Mean 3D IoU drops from 0.82 within 20 m to 0.50 beyond 40 m. The one missed car was 50 m away.
- **Van vs. Car.** One vehicle (4.11 × 1.60 × 1.73 m in KITTI), seen in frames 20 and 30, was labeled Van; KITTI labels it Car. Its 1.73 m height falls between typical car and van heights, and the guideline has no height cutoff between the two classes. In frame 30 its width was 0.31 m too wide.
- **Heading convention.** CVAT and KITTI store box axes differently (length along local x vs. local y). The heading error compares the angle of each box's long side. Front/back direction is not compared.

Full report: [`reports/lidar_eval.md`](reports/lidar_eval.md) · per-box results: [`reports/lidar_matches.csv`](reports/lidar_matches.csv)

## How to run

```bash
python -m venv .venv && source .venv/bin/activate
pip install numpy pandas matplotlib tabulate pytest
python -m pytest -q
python src/evaluate_lidar.py
```

The evaluation expects the KITTI demo project (Supervisely format) in `data/raw/kitti_demo/` and the CVAT export in `annotations/cvat_export/`.
