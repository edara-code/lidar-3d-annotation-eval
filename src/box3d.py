"""3D cuboid geometry: bird's-eye-view (BEV) IoU, 3D IoU, and error metrics.

A box is a dict: {"x", "y", "z", "dx", "dy", "dz", "yaw"}
  x, y, z    center (meters, LiDAR frame: x forward, y left, z up)
  dx, dy, dz box extent along its own x, y, z axes (meters)
  yaw        rotation around the z axis (radians)
"""

import math


def corners_bev(b):
    """Four footprint corners (x, y) of a box, counter-clockwise."""
    c, s = math.cos(b["yaw"]), math.sin(b["yaw"])
    hx, hy = b["dx"] / 2, b["dy"] / 2
    pts = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    return [(b["x"] + px * c - py * s, b["y"] + px * s + py * c) for px, py in pts]


def polygon_area(poly):
    """Area of a polygon given as a list of (x, y) points (shoelace formula)."""
    n = len(poly)
    if n < 3:
        return 0.0
    a = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2


def _clip(subject, a, b):
    """Keep the part of polygon `subject` on the left of the edge a -> b."""
    def inside(p):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0

    def cross_point(p, q):
        x1, y1, x2, y2 = a[0], a[1], b[0], b[1]
        x3, y3, x4, y4 = p[0], p[1], q[0], q[1]
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(den) < 1e-12:
            return q
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    out = []
    n = len(subject)
    for i in range(n):
        cur, prev = subject[i], subject[i - 1]
        if inside(cur):
            if not inside(prev):
                out.append(cross_point(prev, cur))
            out.append(cur)
        elif inside(prev):
            out.append(cross_point(prev, cur))
    return out


def intersection_area_bev(b1, b2):
    """Overlap area of two box footprints (convex polygon clipping)."""
    poly = corners_bev(b1)
    clip = corners_bev(b2)
    for i in range(len(clip)):
        if not poly:
            return 0.0
        poly = _clip(poly, clip[i], clip[(i + 1) % len(clip)])
    return polygon_area(poly)


def iou_bev(b1, b2):
    """IoU of the two footprints seen from above."""
    inter = intersection_area_bev(b1, b2)
    union = b1["dx"] * b1["dy"] + b2["dx"] * b2["dy"] - inter
    return inter / union if union > 0 else 0.0


def iou_3d(b1, b2):
    """3D IoU: footprint overlap times vertical overlap, over the union volume."""
    inter_bev = intersection_area_bev(b1, b2)
    top = min(b1["z"] + b1["dz"] / 2, b2["z"] + b2["dz"] / 2)
    bottom = max(b1["z"] - b1["dz"] / 2, b2["z"] - b2["dz"] / 2)
    inter = inter_bev * max(0.0, top - bottom)
    v1 = b1["dx"] * b1["dy"] * b1["dz"]
    v2 = b2["dx"] * b2["dy"] * b2["dz"]
    union = v1 + v2 - inter
    return inter / union if union > 0 else 0.0


def center_error(b1, b2):
    """Distance between box centers (meters)."""
    return math.dist((b1["x"], b1["y"], b1["z"]), (b2["x"], b2["y"], b2["z"]))


def horizontal_center_error(b1, b2):
    """Center distance seen from above (meters)."""
    return math.dist((b1["x"], b1["y"]), (b2["x"], b2["y"]))


def bottom_offset(b1, b2):
    """Difference in box bottom height, i.e. ground placement (meters)."""
    return abs((b1["z"] - b1["dz"] / 2) - (b2["z"] - b2["dz"] / 2))


def length_width_height(b):
    """Length = longer footprint side, width = shorter side, height = dz."""
    return max(b["dx"], b["dy"]), min(b["dx"], b["dy"]), b["dz"]


def long_axis_angle(b):
    """Direction of the box's long side (radians)."""
    return b["yaw"] if b["dx"] >= b["dy"] else b["yaw"] + math.pi / 2


def angle_diff(a1, a2, period=2 * math.pi):
    """Smallest absolute difference between two angles for the given period."""
    d = (a1 - a2) % period
    return min(d, period - d)


def yaw_axis_error_deg(b1, b2):
    """Heading error of the long axis, ignoring front/back (0-90 degrees)."""
    return math.degrees(angle_diff(long_axis_angle(b1), long_axis_angle(b2), math.pi))


def yaw_direction_error_deg(b1, b2):
    """Heading error including front/back (0-180 degrees)."""
    return math.degrees(angle_diff(long_axis_angle(b1), long_axis_angle(b2)))


def distance_from_sensor(b):
    """Horizontal distance from the LiDAR to the box center (meters)."""
    return math.hypot(b["x"], b["y"])
