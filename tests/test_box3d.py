import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import box3d  # noqa: E402


def box(x=0, y=0, z=0, dx=4, dy=2, dz=1.5, yaw=0.0):
    return {"x": x, "y": y, "z": z, "dx": dx, "dy": dy, "dz": dz, "yaw": yaw}


def test_identical_boxes_iou_is_one():
    b = box(x=10, y=3, yaw=0.7)
    assert box3d.iou_bev(b, b) == pytest.approx(1.0)
    assert box3d.iou_3d(b, b) == pytest.approx(1.0)


def test_half_shift_gives_one_third():
    # Overlap is half of each box: 0.5 / (1 + 1 - 0.5) = 1/3
    assert box3d.iou_bev(box(), box(x=2)) == pytest.approx(1 / 3)


def test_disjoint_boxes_iou_is_zero():
    assert box3d.iou_bev(box(), box(x=20)) == 0.0
    assert box3d.iou_3d(box(), box(z=10)) == 0.0


def test_swapped_dimensions_with_90_degree_turn_is_same_box():
    a = box(dx=4, dy=2, yaw=0.0)
    b = box(dx=2, dy=4, yaw=math.pi / 2)
    assert box3d.iou_3d(a, b) == pytest.approx(1.0)
    assert box3d.yaw_axis_error_deg(a, b) == pytest.approx(0.0, abs=1e-6)
    assert box3d.length_width_height(b) == (4, 2, 1.5)


def test_vertical_half_overlap():
    # Same footprint, half the height overlaps: 0.5 / 1.5 = 1/3
    assert box3d.iou_3d(box(), box(z=0.75)) == pytest.approx(1 / 3)


def test_rotated_45_degrees_overlap():
    a = box(dx=2, dy=2)
    b = box(dx=2, dy=2, yaw=math.pi / 4)
    inter = 8 * (math.sqrt(2) - 1)  # regular octagon inside both squares
    assert box3d.iou_bev(a, b) == pytest.approx(inter / (8 - inter))


def test_yaw_errors_wrap_correctly():
    a = box(yaw=math.radians(170))
    b = box(yaw=math.radians(-170))
    assert box3d.yaw_direction_error_deg(a, b) == pytest.approx(20)
    flipped = box(yaw=math.pi)
    assert box3d.yaw_direction_error_deg(box(), flipped) == pytest.approx(180)
    assert box3d.yaw_axis_error_deg(box(), flipped) == pytest.approx(0, abs=1e-6)


def test_bottom_offset_and_horizontal_error():
    # Same bottom (z - dz/2 = -0.75) despite different heights
    assert box3d.bottom_offset(box(), box(z=0.25, dz=2.0)) == pytest.approx(0)
    assert box3d.bottom_offset(box(), box(z=0.3)) == pytest.approx(0.3)
    assert box3d.horizontal_center_error(box(), box(x=3, y=4, z=9)) == pytest.approx(5)


def test_center_error_and_distance():
    assert box3d.center_error(box(), box(x=3, y=4)) == pytest.approx(5)
    assert box3d.distance_from_sensor(box(x=6, y=8, z=-1)) == pytest.approx(10)
