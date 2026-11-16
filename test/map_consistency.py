#!/usr/bin/env python3
"""Labelled-ROI map metrics in an explicitly declared common coordinate frame."""
import argparse
import json
from pathlib import Path
import numpy as np


def read_pcd(path):
    with Path(path).open("rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD DATA header missing")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key] = values
            if key == "DATA":
                break
        fields = header["FIELDS"]
        counts = [int(n) for n in header.get("COUNT", ["1"] * len(fields))]
        if any(n != 1 for n in counts):
            raise ValueError("only scalar PCD fields supported")
        number = int(header.get("POINTS", header["WIDTH"])[0])
        if number == 0:
            return np.empty((0, 3))
        if header["DATA"][0] == "ascii":
            data = np.loadtxt(stream, ndmin=2)
            if len(data) != number:
                raise ValueError("PCD point count mismatch")
            points = data[:, [fields.index(k) for k in ("x", "y", "z")]]
        elif header["DATA"][0] == "binary":
            kinds = {"F": "f", "I": "i", "U": "u"}
            dtype = np.dtype([(name, "<" + kinds[kind] + size)
                              for name, kind, size in zip(fields, header["TYPE"], header["SIZE"])])
            data = np.fromfile(stream, dtype=dtype, count=number)
            if len(data) != number:
                raise ValueError("truncated binary PCD")
            points = np.column_stack([data[k] for k in ("x", "y", "z")])
        else:
            raise ValueError("binary_compressed PCD unsupported; export ASCII or binary")
    return points[np.isfinite(points).all(axis=1)].astype(float)


def select(points, region):
    low, high = np.asarray(region["min"], float), np.asarray(region["max"], float)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or (high <= low).any():
        raise ValueError("ROI requires finite, ordered 3D bounds")
    return points[((points >= low) & (points <= high)).all(axis=1)]


def cells(points, size):
    return {tuple(row) for row in np.floor(points / size).astype(np.int64)}


def metrics(points, labels, reference=None, voxel=0.1):
    if not np.isfinite(voxel) or voxel <= 0:
        raise ValueError("voxel size must be positive")
    report = {"finite_points": len(points), "static_planes": [], "dynamic_regions": [],
              "note": "ROI and reference quality determine validity; plane fit is not absolute localization accuracy."}
    for region in labels.get("static_planes", []):
        selected = select(points, region)
        row = {"name": region["name"], "points": len(selected), "plane_rms_m": None,
               "normal_error_rad": None, "reference_voxel_retention": None, "plane_fit_valid": False}
        if len(selected) >= 3:
            centered = selected - selected.mean(axis=0)
            values, vectors = np.linalg.eigh(centered.T @ centered / len(selected))
            row["plane_fit_valid"] = bool(values[1] > 1e-8)
            if row["plane_fit_valid"]:
                row["plane_rms_m"] = float(np.sqrt(max(0, values[0])))
            if "normal" in region and row["plane_fit_valid"]:
                normal = np.asarray(region["normal"], float)
                if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) <= 0:
                    raise ValueError("ROI normal must be a finite nonzero 3D vector")
                normal /= np.linalg.norm(normal)
                row["normal_error_rad"] = float(np.arccos(np.clip(abs(vectors[:, 0] @ normal), 0, 1)))
        if reference is not None:
            expected = cells(select(reference, region), voxel)
            row["reference_voxel_retention"] = len(cells(selected, voxel) & expected) / len(expected) if expected else None
        report["static_planes"].append(row)
    for region in labels.get("dynamic_regions", []):
        number = len(select(points, region))
        report["dynamic_regions"].append({"name": region["name"], "points": number,
            "map_fraction": number / len(points) if len(points) else None,
            "expected_empty": region.get("expected_empty", False)})
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--reference")
    parser.add_argument("--labels", required=True)
    parser.add_argument("--frame", required=True)
    parser.add_argument("--reference-frame")
    parser.add_argument("--voxel", type=float, default=0.1)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    labels = json.loads(Path(args.labels).read_text())
    if labels.get("frame") != args.frame or (args.reference and args.reference_frame != args.frame):
        raise ValueError("map, labelled ROI and reference must declare the same frame/segment")
    report = metrics(read_pcd(args.map), labels, read_pcd(args.reference) if args.reference else None, args.voxel)
    report["frame"] = args.frame
    Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
