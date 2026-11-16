#!/usr/bin/env python3
"""Run explicit scene cases and preserve FAIL/SKIPPED rather than hiding gaps."""
import argparse
import json
from pathlib import Path
import subprocess
import signal
import sys
from resource_report import summarize


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--port", type=int, default=11500)
    args = parser.parse_args()
    root = Path(args.output)
    if root.exists():
        raise RuntimeError("Use a new suite output directory")
    root.mkdir(parents=True)
    cases = json.loads(Path(args.manifest).read_text())["cases"]
    reports = []
    for i, case in enumerate(cases):
        name = case["name"]
        if Path(name).name != name or name in (".", ".."):
            raise ValueError("case name must be a simple directory name")
        record = {"name": name, "scene": case["scene"], "labelled": case.get("labelled", False)}
        if not case.get("bag") or not Path(case["bag"]).is_file():
            record.update(status="SKIPPED", reason=case.get("reason", "scene bag unavailable"))
            reports.append(record)
            continue
        directory = root / name
        command = [sys.executable, str(Path(__file__).with_name("replay_data_contract.py")),
                   "--bag", case["bag"], "--port", str(args.port + i), "--output", str(directory)]
        command += case.get("args", [])
        with (root / (name + ".log")).open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                code = process.wait(timeout=case.get("timeout_seconds", 1800))
                record.update(status="PASS" if code == 0 else "FAIL",
                              exit_code=code, metrics=summarize(directory))
            except subprocess.TimeoutExpired:
                record.update(status="FAIL", reason="suite timeout")
            finally:
                if process.poll() is None:
                    # Let the replay's finally block stop its isolated ROS children.
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=60)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                        record.update(status="FAIL", reason="replay cleanup timeout; inspect process log")
        reports.append(record)
        (root / "report.json").write_text(json.dumps({"cases": reports}, indent=2, allow_nan=False) + "\n")
    tested = [r for r in reports if r["status"] != "SKIPPED"]
    report = {"cases": reports, "executed": len(tested),
              "contract_failed_runs": sum(r["status"] == "FAIL" for r in tested),
              "contract_failure_fraction": sum(r["status"] == "FAIL" for r in tested) / len(tested) if tested else None,
              "skipped_scenes": [r["scene"] for r in reports if r["status"] == "SKIPPED"],
              "note": "PASS is a replay/resource contract result, not ground-truth localization success."}
    (root / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 1 if report["contract_failed_runs"] else 0


if __name__ == "__main__":
    sys.exit(main())
