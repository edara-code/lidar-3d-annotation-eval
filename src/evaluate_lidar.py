"""Compare my CVAT 3D cuboids with the official KITTI ground truth.

Usage:
    python src/evaluate_lidar.py

Inputs (Supervisely point cloud format):
    annotations/cvat_export/ds0/ann/*.pcd.json        my boxes, one file per frame
    data/raw/kitti_demo/.../KITTI/annotation.json     ground truth (episode)
    data/raw/kitti_demo/.../KITTI/frame_pointcloud_map.json

Outputs:
    reports/lidar_eval.md       summary tables
    reports/lidar_matches.csv   one row per matched box pair
"""

import csv
import json
import statistics
import sys
from pathlib import Path

from tabulate import tabulate

sys.path.insert(0, str(Path(__file__).parent))
import box3d  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
MY_ANN_DIR = ROOT / "annotations" / "cvat_export" / "ds0" / "ann"
GT_DIR = ROOT / "data" / "raw" / "kitti_demo" / "demo_kitti_pointcloud_episodes_annotated" / "KITTI"
REPORT_MD = ROOT / "reports" / "lidar_eval.md"
REPORT_CSV = ROOT / "reports" / "lidar_matches.csv"

# Ground truth labels vehicles as "Car" (vans included). My Car/Van/Truck boxes
# are all compared against it as "vehicle".
GT_VEHICLE = {"Car"}
MY_VEHICLE = {"Car", "Van", "Truck"}
MATCH_MIN_BEV_IOU = 0.1          # pairs below this are not the same object
IOU_THRESHOLDS = (0.5, 0.7)      # 0.7 is the KITTI standard for cars
DISTANCE_BUCKETS = [(0, 20, "0-20 m"), (20, 40, "20-40 m"), (40, 1e9, "40 m+")]


def to_box(figure):
    g = figure["geometry"]
    p, r, d = g["position"], g["rotation"], g["dimensions"]
    return {"x": p["x"], "y": p["y"], "z": p["z"],
            "dx": d["x"], "dy": d["y"], "dz": d["z"], "yaw": r["z"]}


def frame_name(name):
    """'0000000010.pcd' or '0000000010.pcd.json' -> '0000000010'."""
    return Path(str(name)).name.split(".")[0]


def load_mine():
    files = sorted(MY_ANN_DIR.glob("*.json"))
    if not files:
        sys.exit(f"No annotation files found in {MY_ANN_DIR}")
    frames = {}
    for f in files:
        ann = json.loads(f.read_text())
        classes = {o["key"]: o["classTitle"] for o in ann.get("objects", [])}
        boxes = []
        for fig in ann.get("figures", []):
            if fig.get("geometryType") != "cuboid_3d":
                continue
            boxes.append({"cls": classes.get(fig["objectKey"], "?"), **to_box(fig)})
        frames[frame_name(f.name)] = boxes
    return frames


def load_gt(wanted_frames):
    ann = json.loads((GT_DIR / "annotation.json").read_text())
    fmap = json.loads((GT_DIR / "frame_pointcloud_map.json").read_text())
    index_to_name = {int(k): frame_name(v) for k, v in fmap.items()}
    classes = {o["key"]: o["classTitle"] for o in ann["objects"]}
    frames = {name: [] for name in wanted_frames}
    for fr in ann["frames"]:
        name = index_to_name.get(fr["index"])
        if name not in frames:
            continue
        for fig in fr.get("figures", []):
            if fig.get("geometryType") != "cuboid_3d":
                continue
            frames[name].append({"cls": classes.get(fig["objectKey"], "?"), **to_box(fig)})
    return frames


def match_frame(gt, mine):
    """Greedy one-to-one matching by highest BEV IoU."""
    pairs = []
    for i, g in enumerate(gt):
        for j, m in enumerate(mine):
            iou = box3d.iou_bev(g, m)
            if iou >= MATCH_MIN_BEV_IOU:
                pairs.append((iou, i, j))
    pairs.sort(reverse=True)
    used_g, used_m, matches = set(), set(), []
    for iou, i, j in pairs:
        if i in used_g or j in used_m:
            continue
        used_g.add(i)
        used_m.add(j)
        matches.append((i, j))
    missed = [i for i in range(len(gt)) if i not in used_g]
    extra = [j for j in range(len(mine)) if j not in used_m]
    return matches, missed, extra


def bucket(dist):
    for lo, hi, label in DISTANCE_BUCKETS:
        if lo <= dist < hi:
            return label
    return DISTANCE_BUCKETS[-1][2]


def mean(values):
    return statistics.mean(values) if values else float("nan")


def median(values):
    return statistics.median(values) if values else float("nan")


def main():
    mine_all = load_mine()
    gt_all = load_gt(mine_all.keys())

    per_frame, rows, missed_list, extra_list, other_mine = [], [], [], [], []
    gt_counts, my_counts = {}, {}
    for name in sorted(mine_all):
        gt_frame, my_frame = gt_all[name], mine_all[name]
        for b in gt_frame:
            gt_counts[b["cls"]] = gt_counts.get(b["cls"], 0) + 1
        for b in my_frame:
            my_counts[b["cls"]] = my_counts.get(b["cls"], 0) + 1
        gt = [b for b in gt_frame if b["cls"] in GT_VEHICLE]
        mine = [b for b in my_frame if b["cls"] in MY_VEHICLE]
        other_mine += [(name, b) for b in my_frame if b["cls"] not in MY_VEHICLE]

        matches, missed, extra = match_frame(gt, mine)
        per_frame.append([name, len(gt), len(mine), len(matches), len(missed), len(extra)])
        for i, j in matches:
            g, m = gt[i], mine[j]
            gl, gw, gh = box3d.length_width_height(g)
            ml, mw, mh = box3d.length_width_height(m)
            dist = box3d.distance_from_sensor(g)
            rows.append({
                "frame": name, "my_class": m["cls"],
                "distance_m": round(dist, 1), "bucket": bucket(dist),
                "iou_3d": round(box3d.iou_3d(g, m), 3),
                "iou_bev": round(box3d.iou_bev(g, m), 3),
                "center_err_m": round(box3d.center_error(g, m), 3),
                "center_err_bev_m": round(box3d.horizontal_center_error(g, m), 3),
                "bottom_offset_m": round(box3d.bottom_offset(g, m), 3),
                "len_err_m": round(abs(ml - gl), 3),
                "wid_err_m": round(abs(mw - gw), 3),
                "hgt_err_m": round(abs(mh - gh), 3),
                "yaw_axis_err_deg": round(box3d.yaw_axis_error_deg(g, m), 1),
                "yaw_dir_err_deg": round(box3d.yaw_direction_error_deg(g, m), 1),
                "gt_len": round(gl, 2), "gt_wid": round(gw, 2), "gt_hgt": round(gh, 2),
                "my_len": round(ml, 2), "my_wid": round(mw, 2), "my_hgt": round(mh, 2),
            })
        missed_list += [(name, round(box3d.distance_from_sensor(gt[i]), 1)) for i in missed]
        extra_list += [(name, mine[j]["cls"], round(box3d.distance_from_sensor(mine[j]), 1))
                       for j in extra]

    total_gt = sum(r[1] for r in per_frame)
    total_mine = sum(r[2] for r in per_frame)
    n = len(rows)
    out = []

    def section(title, table, headers):
        text = f"## {title}\n\n" + tabulate(table, headers=headers, tablefmt="github") + "\n"
        out.append(text)
        print(text)

    section("Boxes by class (all frames)",
            [[c, gt_counts.get(c, 0), my_counts.get(c, 0)]
             for c in sorted(set(gt_counts) | set(my_counts))],
            ["Class", "Ground truth", "Mine"])

    section("Per frame (vehicles)", per_frame,
            ["Frame", "GT vehicles", "My vehicles", "Matched", "Missed", "Extra"])

    summary = [
        ["Frames with ground-truth vehicles", f"{sum(r[1] > 0 for r in per_frame)} of {len(per_frame)}"],
        ["Ground-truth vehicles", total_gt],
        ["My vehicle boxes", total_mine],
        ["Matched pairs", n],
        ["Recall (matched / GT)", f"{n / total_gt:.1%}" if total_gt else "n/a"],
    ]
    for t in IOU_THRESHOLDS:
        hits3d = sum(r["iou_3d"] >= t for r in rows)
        hitsbev = sum(r["iou_bev"] >= t for r in rows)
        summary.append([f"Recall at 3D IoU >= {t}", f"{hits3d / total_gt:.1%}" if total_gt else "n/a"])
        summary.append([f"Recall at BEV IoU >= {t}", f"{hitsbev / total_gt:.1%}" if total_gt else "n/a"])
    section("Summary", summary, ["Metric", "Value"])

    def col(key):
        return [r[key] for r in rows]

    metric_rows = []
    for label, key in [("3D IoU", "iou_3d"), ("BEV IoU", "iou_bev"),
                       ("Center error (m)", "center_err_m"),
                       ("Center error, seen from above (m)", "center_err_bev_m"),
                       ("Bottom offset, ground placement (m)", "bottom_offset_m"),
                       ("Length error (m)", "len_err_m"), ("Width error (m)", "wid_err_m"),
                       ("Height error (m)", "hgt_err_m"),
                       ("Yaw error, axis (deg)", "yaw_axis_err_deg")]:
        metric_rows.append([label, f"{mean(col(key)):.3f}", f"{median(col(key)):.3f}"])
    section("Matched-box quality", metric_rows, ["Metric", "Mean", "Median"])

    dist_rows = []
    gt_by_bucket = {}
    for name in mine_all:
        for b in gt_all[name]:
            if b["cls"] in GT_VEHICLE:
                lab = bucket(box3d.distance_from_sensor(b))
                gt_by_bucket[lab] = gt_by_bucket.get(lab, 0) + 1
    for _, _, lab in DISTANCE_BUCKETS:
        sub = [r for r in rows if r["bucket"] == lab]
        g = gt_by_bucket.get(lab, 0)
        dist_rows.append([lab, g, len(sub),
                          f"{len(sub) / g:.0%}" if g else "n/a",
                          f"{mean([r['iou_3d'] for r in sub]):.3f}" if sub else "n/a",
                          f"{mean([r['center_err_m'] for r in sub]):.3f}" if sub else "n/a"])
    section("By distance from the sensor", dist_rows,
            ["Distance", "GT vehicles", "Matched", "Recall", "Mean 3D IoU", "Mean center error (m)"])

    gt_x_long = [b["dx"] >= b["dy"] for name in mine_all for b in gt_all[name] if b["cls"] in GT_VEHICLE]
    my_x_long = [b["dx"] >= b["dy"] for name in mine_all for b in mine_all[name] if b["cls"] in MY_VEHICLE]
    share_gt = mean(gt_x_long) if gt_x_long else float("nan")
    share_my = mean(my_x_long) if my_x_long else float("nan")
    heading_rows = [
        ["GT boxes with length along local x", f"{share_gt:.0%}"],
        ["My boxes with length along local x", f"{share_my:.0%}"],
    ]
    if abs(share_gt - share_my) < 0.5:
        flipped = sum(r["yaw_dir_err_deg"] > 90 for r in rows)
        heading_rows.append(["Matched pairs facing opposite ways (>90 deg)", f"{flipped} of {n}"])
    else:
        heading_rows.append(["Front/back direction",
                             "not compared (the two tools use different box axis conventions)"])
    section("Heading check", heading_rows, ["Check", "Value"])

    section("Matched pairs",
            [[r["frame"], r["my_class"], r["distance_m"], r["iou_3d"], r["iou_bev"],
              r["center_err_bev_m"], r["bottom_offset_m"], r["hgt_err_m"], r["yaw_axis_err_deg"]]
             for r in rows],
            ["Frame", "My class", "Dist (m)", "3D IoU", "BEV IoU", "Center err, top (m)",
             "Bottom offset (m)", "Height err (m)", "Yaw err (deg)"])

    van_rows = [r for r in rows if r["my_class"] != "Car"]
    if van_rows:
        section("Matched boxes I labeled as a class other than Car",
                [[r["frame"], r["my_class"], r["my_len"], r["my_wid"], r["my_hgt"],
                  r["gt_len"], r["gt_wid"], r["gt_hgt"]] for r in van_rows],
                ["Frame", "My class", "My L", "My W", "My H", "GT L", "GT W", "GT H"])

    section("Missed ground-truth vehicles", missed_list or [["none", ""]], ["Frame", "Distance (m)"])
    section("My boxes with no ground-truth match", extra_list or [["none", "", ""]],
            ["Frame", "My class", "Distance (m)"])
    if other_mine:
        section("My non-vehicle boxes (no ground truth for these classes)",
                [[f, b["cls"], round(box3d.distance_from_sensor(b), 1)] for f, b in other_mine],
                ["Frame", "Class", "Distance (m)"])

    REPORT_MD.parent.mkdir(exist_ok=True)
    REPORT_MD.write_text("# LiDAR annotation evaluation\n\n" + "\n".join(out))
    if rows:
        with REPORT_CSV.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(f"Saved {REPORT_MD.relative_to(ROOT)} and {REPORT_CSV.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
