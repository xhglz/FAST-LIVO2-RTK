#!/usr/bin/env python3
"""Summarize measured resource/validity contracts; never infer scene accuracy."""
import argparse
import csv
import json
import math
from pathlib import Path


def rows(path):
    if not path.exists():
        return []
    with path.open() as stream:
        return list(csv.DictReader(stream))


def distribution(values):
    values = sorted(float(x) for x in values if math.isfinite(float(x)))
    if not values:
        return {"count": 0, "p95": None, "p99": None, "max": None}
    def percentile(q):
        index = (len(values) - 1) * q
        lo = int(index)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)
    return {"count": len(values), "p95": percentile(0.95), "p99": percentile(0.99), "max": values[-1]}


def memory_metrics(samples, warmup=10.0):
    active = [r for r in samples if float(r["sim_time"]) > 0]
    if not active:
        return {"samples": 0, "peak_rss_mb": None, "steady_growth_mb": None}
    steady = [r for r in active if float(r["sim_time"]) >= float(active[0]["sim_time"]) + warmup]
    metric = {"samples": len(active), "peak_rss_mb": max(float(r["rss_kb"]) for r in active) / 1024,
              "kernel_peak_rss_mb": max(float(r["hwm_kb"]) for r in active) / 1024 if "hwm_kb" in active[0] else None,
              "warmup_sim_seconds": warmup, "steady_samples": len(steady),
              "steady_growth_mb": None, "slope_mb_per_wall_min": None, "slope_mb_per_sim_min": None}
    if len(steady) >= 2:
        y = [float(r["rss_kb"]) / 1024 for r in steady]
        metric["steady_growth_mb"] = y[-1] - y[0]
        for field, name in (("wall_time", "wall"), ("sim_time", "sim")):
            x = [float(r[field]) - float(steady[0][field]) for r in steady]
            xm, ym = sum(x) / len(x), sum(y) / len(y)
            denom = sum((v - xm) ** 2 for v in x)
            metric["slope_mb_per_" + name + "_min"] = (
                sum((a - xm) * (b - ym) for a, b in zip(x, y)) / denom * 60 if denom > 0 else None)
    tail = [r for r in active if float(r["sim_time"]) >= float(active[-1]["sim_time"]) - 30]
    metric["tail_growth_mb"] = None
    metric["tail_sim_seconds"] = None
    if len(tail) >= 2:
        metric["tail_growth_mb"] = (float(tail[-1]["rss_kb"]) - float(tail[0]["rss_kb"])) / 1024
        metric["tail_sim_seconds"] = float(tail[-1]["sim_time"]) - float(tail[0]["sim_time"])
    return metric


def paired_processing(data):
    paired = {}
    for row in data:
        stages = paired.setdefault(row["time"], {})
        stages.setdefault(row["stage"], []).append(float(row["processing_ms"]))
    return distribution(stages["LIO"][0] + stages["VIO"][0] for stages in paired.values()
                        if len(stages.get("LIO", [])) == 1 and len(stages.get("VIO", [])) == 1)


def validity_metrics(status):
    status = sorted(status, key=lambda r: r["time"])
    first = next((i for i, r in enumerate(status) if r["valid"]), None)
    if first is None:
        return {"initialized": False, "loss_episodes": None, "invalid_time_fraction": None, "recovery_seconds": []}
    status = status[first:]
    episodes, current = [], None
    invalid_time = 0.0
    lost = 0
    previous_state = status[0]["state"]
    for i, r in enumerate(status):
        if r["state"] == 3 and previous_state != 3:
            lost += 1
        previous_state = r["state"]
        if not r["valid"] and current is None:
            current = {"start": r["time"], "reason": r.get("reason", ""), "censored": True}
        if r["valid"] and current is not None:
            current.update(end=r["time"], seconds=r["time"] - current["start"], censored=False)
            episodes.append(current)
            current = None
        if i + 1 < len(status) and not r["valid"]:
            invalid_time += max(0.0, status[i + 1]["time"] - r["time"])
    if current is not None:
        episodes.append(current)
    duration = status[-1]["time"] - status[0]["time"]
    recovered = [r["seconds"] for r in episodes if not r["censored"]]
    return {"initialized": True, "loss_episodes": lost, "observed_sim_seconds": duration,
            "loss_episodes_per_min": lost * 60 / duration if duration > 0 else None,
            "invalid_time_fraction": invalid_time / duration if duration > 0 else None,
            "invalid_intervals": episodes, "recovery_seconds": recovered,
            "recovery_distribution": distribution(recovered), "final_valid": status[-1]["valid"]}


def summarize(directory):
    directory = Path(directory)
    result_file = directory / "result.json"
    result = json.loads(result_file.read_text()) if result_file.exists() else {"errors": ["missing replay result"]}
    frontend = rows(directory / "mapper/debug/timing.csv")
    backend = rows(directory / "mapper/online/resources.csv")
    memory = rows(directory / "rss.csv")
    quality = rows(directory / "mapper/debug/frontend_quality.csv")
    state_file = directory / "safety_status.json"
    status = json.loads(state_file.read_text()) if state_file.exists() else []
    measured = {}
    for stage in ("LIO", "VIO"):
        measured[stage + "_processing_ms"] = distribution(r["processing_ms"] for r in frontend if r["stage"] == stage)
        measured[stage + "_measurement_age_ms"] = distribution(r["measurement_age_ms"] for r in frontend if r["stage"] == stage)
    for stage in ("keyframe", "gnss"):
        measured["backend_" + stage + "_processing_ms"] = distribution(
            r["processing_ms"] for r in backend if r["type"] == stage)
    measured["backend_queue_wait_ms"] = distribution(r["queue_wait_ms"] for r in backend)
    measured["paired_LIVO_processing_ms"] = paired_processing(frontend)
    capacity = {}
    for label, data in (("frontend", rows(directory / "mapper/debug/resources.csv")), ("backend", backend)):
        if data:
            keys = [k for k in data[0] if k not in ("time", "segment", "stage", "type", "processing_ms", "queue_wait_ms")]
            capacity[label + "_peaks"] = {k: max(float(r[k]) for r in data) for k in keys}
    accepted = [r for r in quality if r["accepted"] == "1"]
    return {"directory": str(directory), "run": result.get("run", {}), "contract_pass": not result.get("errors"),
            "data_contract": {k: result.get(k) for k in ("body_clouds", "geometry_checks", "online", "dynamic_filter")},
            "errors": result.get("errors", []), "memory": memory_metrics(memory),
            "latency": measured, "validity": validity_metrics(status), "capacity": capacity,
            "map_consistency_proxies": {
                "accepted_normalized_rms": distribution(r["normalized_rms"] for r in accepted),
                "accepted_coverage": distribution(r["coverage"] for r in accepted),
                "static_wall_retention": None, "dynamic_region_residual": None,
                "absolute_pose_error": None,
                "note": "Residual/coverage are frontend self-consistency, not independent map accuracy. ROI labels/ground truth required."},
            "limits": "Replay-scaled ROS age is not physical end-to-end wall latency. Short RSS slopes do not establish long-term boundedness."}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directories", nargs="+")
    parser.add_argument("--output", required=True)
    parser.add_argument("--suite-report")
    parser.add_argument("--baseline-replay")
    args = parser.parse_args()
    reports = [summarize(d) for d in args.directories]
    report = {"runs": reports, "contract_failed_runs": sum(not r["contract_pass"] for r in reports),
              "run_count": len(reports)}
    if args.suite_report:
        report["scene_suite"] = json.loads(Path(args.suite_report).read_text())
    if args.baseline_replay:
        baseline = json.loads(Path(args.baseline_replay).read_text())
        report["disabled_baseline"] = {k: baseline.get(k) for k in ("body_clouds", "max_pose_error", "errors")}
    Path(args.output).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"run_count": len(reports), "contract_failed_runs": report["contract_failed_runs"],
                      "report": args.output}, indent=2))


if __name__ == "__main__":
    main()
