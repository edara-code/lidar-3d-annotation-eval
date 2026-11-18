# LiDAR 3D Cuboid Annotation Guidelines

Guidelines for labeling objects in LiDAR point clouds from driving scenes with 3D cuboids.

## Classes

| Class | Includes | Excludes |
|---|---|---|
| **Car** | Sedans, hatchbacks, SUVs, small pickups | Vans, trucks, trailers |
| **Van** | Minivans, delivery vans, camper vans | Large box trucks |
| **Truck** | Box trucks, lorries, semi-trucks, large pickups with cargo beds | Buses, trams |
| **Pedestrian** | People walking, standing, or sitting | People riding bicycles |
| **Cyclist** | Person and bicycle together, in one box | Parked bicycles with no rider |

If you are unsure between two vehicle classes, choose the smaller class and add a comment.

## What to label

- Label every object of the five classes that has **at least 10 LiDAR points** on it.
- Label parked and moving objects alike.
- Do not label objects with fewer than 10 points. They cannot be fitted reliably.
- Do not label reflections, ghost points, or objects seen only in the camera image.

## Fitting the cuboid

1. **Tight fit.** The box touches the outermost points of the object on every side. Do not leave empty margins.
2. **Full extent.** If part of the object is hidden, size the box to the object's real size, not just the visible points. Use the camera image and typical sizes to estimate it.
3. **Ground contact.** The bottom of the box sits on the ground surface, not below it and not floating above it.
4. **Exclude extras.** Mirrors and antennas are included only if they are part of the solid outline. Do not include the ground, nearby poles, or vegetation.

### Typical sizes (length × width × height, meters)

| Class | Typical size |
|---|---|
| Car | 3.9 × 1.6 × 1.5 |
| Van | 5.0 × 1.9 × 2.2 |
| Truck | 7–12 × 2.5 × 3.0 |
| Pedestrian | 0.8 × 0.6 × 1.7 |
| Cyclist | 1.8 × 0.6 × 1.7 |

These are guides for estimating hidden parts. Always fit the visible points first.

## Heading (yaw)

- The box's front faces the direction the object is facing (or moving).
- For vehicles, use the front of the vehicle (windshield and hood side).
- For pedestrians, use the direction they are facing or walking.
- For cyclists, use the direction of the bicycle.
- Check the heading in the top (bird's-eye) view. A box rotated 180° has the right footprint but the wrong heading.

## Occlusion

Mark an object as **occluded** when another object or structure hides a noticeable part of it (roughly 30% or more) from the sensor. Still fit the full box.

## Consistency across frames

- Keep the size of the same object the same in every frame where it appears.
- Re-check heading for objects that turn.

## Quality checks before submitting a frame

- Every qualifying object has a box.
- Each box has the correct class.
- Bottoms touch the ground, and box edges touch the outermost points.
- Headings point the right way in the top view.
- Occluded objects are marked.
