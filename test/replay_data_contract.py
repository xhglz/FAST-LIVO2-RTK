#!/usr/bin/env python3
"""Short, isolated bag replay for frame/time contracts and local-pose regression."""
import argparse
import csv
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client


def wait_for(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if predicate():
                return
        except (OSError, xmlrpc.client.Error):
            pass
        time.sleep(0.1)
    raise RuntimeError("Timeout: " + description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--config", default="HH-LVGO.yaml")
    parser.add_argument("--camera-config", default="camera_HH-LVGO-01.yaml")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=1.0, help="Bag playback rate")
    parser.add_argument("--gps", choices=("on", "off"), default="off")
    parser.add_argument("--image", choices=("on", "off"), default="on")
    parser.add_argument("--port", type=int, default=11331)
    parser.add_argument("--output")
    parser.add_argument("--reference", help="Previous result.json for raw local-pose comparison")
    parser.add_argument("--binary", help="Optional archived baseline executable")
    parser.add_argument("--library-dir", help="Libraries belonging to the archived baseline")
    parser.add_argument("--legacy", action="store_true", help="Capture baseline; skip new contract checks")
    parser.add_argument("--backend-tests", action="store_true", help="Run backend gtests under this master")
    parser.add_argument("--health-check", action="store_true", help="Check diagnostics and frontend metrics")
    parser.add_argument("--faults", action="store_true", help="Inject GNSS jump, dropout, reverse time and FLOAT")
    parser.add_argument("--enforce-gnss", action="store_true", help="Enable trusted FIX gate")
    parser.add_argument("--online", action="store_true", help="Use iSAM2; never send Enter")
    parser.add_argument("--expect-uninitialized", action="store_true", help="Verify alignment stays unavailable")
    parser.add_argument("--deterministic", action="store_true", help="Limit mapper OpenMP/BLAS threads for strict replay comparison")
    parser.add_argument("--dynamic-filter", action="store_true", help="Enable geometric dynamic filtering and check its diagnostics")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.faults and (args.gps != "on" or args.duration < 18):
        parser.error("--faults requires --gps on and --duration >= 18")
    if args.online and args.faults and args.duration < 38:
        parser.error("online fault checks need --duration >= 38 (faults occur after 25 seconds)")
    output = Path(args.output) if args.output else Path(tempfile.mkdtemp(prefix="livo-contract-"))
    output.mkdir(parents=True, exist_ok=True)
    if (output / "result.json").exists():
        raise RuntimeError("Use a new output directory to preserve previous results")
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", args.port))
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % args.port
    os.environ["ROS_HOME"] = str(output / "ros")
    children, logs = [], []

    def start(command, name, **kwargs):
        log = (output / (name + ".log")).open("w")
        logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, **kwargs)
        children.append(proc)
        return proc

    try:
        start(["roscore", "-p", str(args.port)], "master")
        master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
        wait_for(lambda: master.getPid("/contract_test")[0] == 1, 20, "ROS master")
        import numpy as np
        import rosparam
        import rospkg
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs import point_cloud2
        from diagnostic_msgs.msg import DiagnosticArray
        from gnss_comm.msg import GnssPVTSolnMsg
        from tf2_msgs.msg import TFMessage
        from tf.transformations import quaternion_matrix
        package = Path(rospkg.RosPack().get_path("fast_livo"))
        if args.backend_tests:
            tests = start(["rosrun", "fast_livo", "backend_events_test"], "backend_tests")
            tests.wait(timeout=60)
            if tests.returncode != 0:
                raise RuntimeError("Backend regression tests failed; inspect backend_tests.log")
        for parameters, namespace in rosparam.load_file(str(package / "config" / args.config)):
            rosparam.upload_params(namespace, parameters)
        if not args.legacy:
            for parameters, namespace in rosparam.load_file(str(package / "config/dynamic_filter.yaml")):
                rosparam.upload_params(namespace, parameters)
            for parameters, namespace in rosparam.load_file(str(package / "config/health.yaml")):
                rosparam.upload_params(namespace, parameters)
            if args.online:
                for parameters, namespace in rosparam.load_file(str(package / "config/online_backend.yaml")):
                    rosparam.upload_params(namespace, parameters)
        for parameters, _ in rosparam.load_file(str(package / "config" / args.camera_config)):
            rosparam.upload_params("/laserMapping", parameters)
        rospy.init_node("data_contract_observer", anonymous=True, disable_signals=True)
        rospy.set_param("/use_sim_time", True)
        rospy.set_param("/gps/gps_en", args.gps == "on")
        rospy.set_param("/common/img_en", 1 if args.image == "on" else 0)
        rospy.set_param("/gps/debug_mode", False)
        rospy.set_param("/opt/debug_mode", False)
        rospy.set_param("/pcd_save/pcd_save_en", False)
        rospy.set_param("/pcd_save/colmap_output_en", False)
        rospy.set_param("/evo/pose_output_en", False)
        rospy.set_param("/health/enforce_gnss_fix", args.enforce_gnss)
        rospy.set_param("/backend/mode", "online" if args.online else "batch")
        rospy.set_param("/dynamic_filter/enabled", args.dynamic_filter)
        rospy.set_param("/laserMapping/outputfilepath", str(output / "mapper"))
        poses, body_clouds, world_clouds, gps_headers = {}, {}, {}, []
        errors, lock = [], threading.Lock()
        diagnostic_history, injections = [], []
        global_poses, corrections, global_maps = {}, {}, []
        proxy_state = {"start": None, "done": set()}
        odom_frame = rospy.get_param("/frames/odom", "odom")
        imu_frame = rospy.get_param("/frames/imu", "imu_link")
        map_frame = rospy.get_param("/frames/map", "map")
        gnss_frame = rospy.get_param("/frames/gnss", "gnss_link")

        def pose_callback(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            with lock:
                poses[msg.header.stamp.to_nsec()] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if not args.legacy and (msg.header.frame_id != odom_frame or msg.child_frame_id != imu_frame):
                    errors.append("odometry frame mismatch")

        def cloud_callback(msg, destination, expected):
            if args.dynamic_filter and msg.width * msg.height == 0:
                return
            sample = next(point_cloud2.read_points(msg, field_names=("x", "y", "z")), None)
            with lock:
                destination[msg.header.stamp.to_nsec()] = sample
                if not args.legacy and msg.header.frame_id != expected:
                    errors.append("cloud frame mismatch")

        def gps_callback(msg):
            with lock:
                gps_headers.append(msg.header.stamp.to_nsec())
                if not args.legacy and (msg.header.frame_id != map_frame or msg.child_frame_id != gnss_frame):
                    errors.append("GNSS frame mismatch")

        def diagnostics_callback(msg):
            with lock:
                diagnostic_history.extend(
                    {"time": msg.header.stamp.to_sec(), "name": s.name,
                     "level": s.level, "message": s.message,
                     "values": {kv.key: kv.value for kv in s.values}}
                    for s in msg.status)

        def global_callback(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            with lock:
                global_poses[msg.header.stamp.to_nsec()] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if msg.header.frame_id != map_frame or msg.child_frame_id != imu_frame:
                    errors.append("global odometry frame mismatch")
                if not all(math.isfinite(x) for x in msg.pose.covariance):
                    errors.append("global covariance nonfinite")

        def transform_callback(msg):
            with lock:
                for transform in msg.transforms:
                    if transform.header.frame_id == map_frame and transform.child_frame_id == odom_frame:
                        t, q = transform.transform.translation, transform.transform.rotation
                        corrections[transform.header.stamp.to_nsec()] = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

        def map_callback(msg):
            with lock:
                global_maps.append(msg.width * msg.height)
                if msg.header.frame_id != map_frame:
                    errors.append("global map frame mismatch")

        gps_topic = rospy.get_param("/gps/gps_topic", "/ublox_driver/receiver_pvt")
        proxy_pub = rospy.Publisher(gps_topic, GnssPVTSolnMsg, queue_size=100)

        def fault_callback(msg):
            measurement = 315964800 + msg.time.week * 604800 + msg.time.tow - 18
            if proxy_state["start"] is None:
                proxy_state["start"] = measurement
            elapsed = measurement - proxy_state["start"]
            if args.online:
                elapsed -= 20.0
            if 8 <= elapsed < 10:
                if "dropout" not in proxy_state["done"]:
                    injections.append("dropout")
                    proxy_state["done"].add("dropout")
                return
            modified = copy.deepcopy(msg)
            for name, at in (("jump", 5), ("reverse", 12), ("float", 14)):
                if elapsed >= at and name not in proxy_state["done"]:
                    proxy_state["done"].add(name)
                    injections.append(name)
                    if name == "jump":
                        modified.latitude += 0.001
                    elif name == "reverse":
                        modified.time.tow -= 0.5
                    else:
                        modified.carr_soln = 1
                    break
            proxy_pub.publish(modified)

        subscribers = [
            rospy.Subscriber("/odometry/fast_livo2", Odometry, pose_callback, queue_size=1000),
            rospy.Subscriber("/synced_cloud", PointCloud2,
                             lambda msg: cloud_callback(msg, body_clouds, imu_frame), queue_size=1000),
            rospy.Subscriber("/cloud_registered", PointCloud2,
                             lambda msg: cloud_callback(msg, world_clouds, odom_frame), queue_size=1000),
            rospy.Subscriber("/gps/odometry", Odometry, gps_callback, queue_size=1000)]
        if not args.legacy:
            subscribers.append(rospy.Subscriber("/diagnostics", DiagnosticArray,
                                                diagnostics_callback, queue_size=1000))
        if args.faults:
            subscribers.append(rospy.Subscriber("/health_test/gnss", GnssPVTSolnMsg,
                                                fault_callback, queue_size=1000))
        if args.online:
            subscribers.extend([
                rospy.Subscriber("/odometry/global", Odometry, global_callback, queue_size=1000),
                rospy.Subscriber("/tf", TFMessage, transform_callback, queue_size=1000),
                rospy.Subscriber("/map/global", PointCloud2, map_callback, queue_size=10)])
        env = os.environ.copy()
        if args.deterministic:
            env.update(OMP_THREAD_LIMIT="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
        if args.library_dir:
            env["LD_LIBRARY_PATH"] = args.library_dir + ":" + env.get("LD_LIBRARY_PATH", "")
        command = [args.binary] if args.binary else ["rosrun", "fast_livo", "fastlivo_mapping"]
        mapper = start(command, "mapper", stdin=subprocess.PIPE, env=env)
        start(["rosrun", "image_transport", "republish", "compressed",
               "in:=/left_camera/image", "raw", "out:=/left_camera/image"], "image")
        wait_for(lambda: subscribers[0].get_num_connections() > 0, 30, "mapper publisher")
        play_command = ["rosbag", "play", args.bag, "--clock", "--duration", str(args.duration),
                        "--delay", "1", "--queue", "1000", "--rate", str(args.rate)]
        if args.faults:
            wait_for(lambda: proxy_pub.get_num_connections() > 0, 30, "GNSS proxy connection")
            play_command.append(gps_topic + ":=/health_test/gnss")
        playback = start(play_command, "playback")
        playback.wait(timeout=args.duration / args.rate + 60)
        if playback.returncode != 0 or mapper.poll() is not None:
            raise RuntimeError("Playback or mapper failed; inspect process logs")
        # A brief drain period is needed because bag completion is not frontend completion.
        last_size = [-1]
        last_change = [time.monotonic()]
        def drained():
            with lock:
                size = len(poses)
            if size != last_size[0]:
                last_size[0] = size
                last_change[0] = time.monotonic()
            return size > 0 and time.monotonic() - last_change[0] >= 1.0
        wait_for(drained, 30, "frontend drain")
        if not args.legacy and not args.online:
            mapper.stdin.write(b"\n")
            mapper.stdin.flush()
            manifest = output / "mapper/debug/keyframe_events.csv"
            wait_for(lambda: manifest.exists() and manifest.stat().st_size > 100, 30, "event export")
        with lock:
            snapshot = dict(poses)
            body = dict(body_clouds)
            world = dict(world_clouds)
        if not snapshot or not body:
            raise RuntimeError("No local poses or body clouds")
        geometry_checks = 0
        if not args.legacy:
            for stamp, point in body.items():
                if stamp not in snapshot:
                    errors.append("body cloud has no exact-time pose")
                    continue
                if stamp in world and point is not None and world[stamp] is not None:
                    pose = snapshot[stamp]
                    transformed = quaternion_matrix(pose[3:])[:3, :3].dot(point) + pose[:3]
                    if np.linalg.norm(transformed - world[stamp]) > 1e-3:
                        errors.append("body-to-odom geometry mismatch")
                    geometry_checks += 1
            if not args.online:
                with manifest.open() as stream:
                    events = list(csv.DictReader(stream))
                if len(events) != len(body):
                    errors.append("keyframe event/cloud count mismatch")
                if any(float(b["time"]) <= float(a["time"]) for a, b in zip(events, events[1:])):
                    errors.append("keyframe measurement time is not strictly increasing")
            if args.gps == "off" and gps_headers:
                errors.append("GNSS output remains active with gps_en=false")
            if args.gps == "on" and not gps_headers:
                errors.append("No GNSS output with gps_en=true")
            if geometry_checks == 0:
                errors.append("No matched cloud geometry checks")
        ordered = [{"stamp_ns": stamp, "pose": snapshot[stamp]} for stamp in sorted(snapshot)]
        max_pose_error = None
        if args.reference:
            reference = json.loads(Path(args.reference).read_text())["poses"]
            if [r["stamp_ns"] for r in reference] != [r["stamp_ns"] for r in ordered]:
                errors.append("reference/current timestamp sequences differ")
            else:
                max_pose_error = max(float(np.max(np.abs(np.array(a["pose"]) - b["pose"])))
                                     for a, b in zip(reference, ordered))
                if max_pose_error > 1e-5:
                    errors.append("local pose regression exceeds 1e-5")
        result = {"poses": ordered, "body_clouds": len(body), "gnss_messages": len(gps_headers),
                  "geometry_checks": geometry_checks, "max_pose_error": max_pose_error,
                  "errors": sorted(set(errors))}
        if args.dynamic_filter:
            with (output / "mapper/debug/dynamic_quality.csv").open() as stream:
                dynamic_rows = list(csv.DictReader(stream))
            if not dynamic_rows:
                errors.append("missing dynamic filter diagnostics")
            elif not any(int(r["temporary_points"]) > 0 for r in dynamic_rows):
                errors.append("new-region temporary layer never bootstrapped")
            if dynamic_rows and not any(int(r["confirmed"]) > 0 for r in dynamic_rows):
                errors.append("no points confirmed static")
            if any(int(r["cells"]) > 60000 or int(r["temporary_points"]) > 20000 for r in dynamic_rows):
                errors.append("dynamic filter resource budget exceeded")
            result["dynamic_filter"] = {
                "frames": len(dynamic_rows),
                "dynamic_point_classifications": sum(int(r["dynamic"]) for r in dynamic_rows),
                "moving_cluster_detections": sum(int(r["moving_clusters"]) for r in dynamic_rows),
                "removed_cells": sum(int(r["removed"]) for r in dynamic_rows),
                "frozen_frames": sum(r["reliable"] == "0" for r in dynamic_rows),
                "last": dynamic_rows[-1] if dynamic_rows else None,
            }
        if args.health_check or args.faults or args.online or args.dynamic_filter:
            with (output / "mapper/debug/gnss_quality.csv").open() as stream:
                gnss_rows = list(csv.DictReader(stream))
            with (output / "mapper/debug/frontend_quality.csv").open() as stream:
                frontend_rows = list(csv.DictReader(stream))
            if not diagnostic_history or not frontend_rows:
                errors.append("missing diagnostics or frontend metrics")
            metric_keys = ["coverage", "normalized_rms", "eigen_ratio"] + [
                "H%d%d" % (i, j) for i in range(6) for j in range(6)]
            if any(not math.isfinite(float(row[key])) for row in frontend_rows for key in metric_keys):
                errors.append("nonfinite frontend diagnostic metrics")
            if args.health_check and not args.faults and any(row["level"] == 2 for row in diagnostic_history):
                errors.append("normal replay contains DEGRADED diagnostics")
            classes = sorted(set(row["classification"] for row in gnss_rows))
            if args.faults:
                if set(injections) != {"jump", "dropout", "reverse", "float"}:
                    errors.append("not all faults injected")
                if not {"POSITION_JUMP", "FLOAT"}.issubset(classes):
                    errors.append("GNSS jump/FLOAT not diagnosed")
                messages = "|".join(row["message"] for row in diagnostic_history)
                if "TIME_REVERSED" not in messages or "MISSING" not in messages:
                    errors.append("reverse/dropout not diagnosed")
                gps_states = [row for row in diagnostic_history
                              if row["name"] == "fast_livo/input/gnss"]
                if not gps_states or gps_states[-1]["message"] != "NORMAL:OK":
                    errors.append("GNSS input did not recover after faults")
                if not args.enforce_gnss and any(row["forwarded"] != "1" for row in gnss_rows):
                    errors.append("advisory mode rejected quality-only GNSS faults")
            if args.enforce_gnss:
                if any(row["forwarded"] == "1" and
                       (row["trusted"] != "1" or row["stable"] != "1") for row in gnss_rows):
                    errors.append("untrusted GNSS forwarded by gate")
                if not any(row["forwarded"] == "1" for row in gnss_rows):
                    errors.append("GNSS gate never acquired FIX")
            if not args.online:
                with (output / "mapper/debug/gnss_events.csv").open() as stream:
                    backend_gnss = list(csv.DictReader(stream))
                if len(backend_gnss) != sum(row["forwarded"] == "1" for row in gnss_rows):
                    errors.append("GNSS forwarded/backend event count mismatch")
            result["health"] = {
                "classes": classes, "injections": injections,
                "frontend_rows": len(frontend_rows),
                "error_diagnostics": sum(row["level"] == 2 for row in diagnostic_history),
                "forwarded": sum(row["forwarded"] == "1" for row in gnss_rows)}
            (output / "diagnostics.json").write_text(json.dumps(diagnostic_history, indent=2) + "\n")
            result["errors"] = sorted(set(errors))
        if args.online:
            with (output / "mapper/online/events.csv").open() as stream:
                online_events = list(csv.DictReader(stream))
            with (output / "mapper/online/gnss_factors.csv").open() as stream:
                factor_times = [float(row["measurement_time"]) for row in csv.DictReader(stream)]
            online_status = [row for row in diagnostic_history if row["name"] == "fast_livo/online_backend"]
            expect_ready = args.gps == "on" and not args.expect_uninitialized
            if not online_status:
                errors.append("no online backend diagnostics")
            if expect_ready and (not global_poses or not global_maps or not factor_times):
                errors.append("online alignment/pose/map/factors unavailable")
            if not expect_ready and (global_poses or corrections or factor_times):
                errors.append("global output fabricated without alignment")
            if len(set(factor_times)) != len(factor_times):
                errors.append("GNSS measurement reused as multiple factors")
            quality = {float(row["time"]): row for row in gnss_rows}
            if any(t not in quality or quality[t]["trusted"] != "1" or
                   quality[t]["stable"] != "1" for t in factor_times):
                errors.append("factor does not correspond to a genuine trusted FIX")
            global_checks = 0
            for stamp, pose in global_poses.items():
                if stamp not in corrections or stamp not in snapshot:
                    errors.append("global output lacks exact-time local pose/correction")
                    continue
                correction = corrections[stamp]
                T = quaternion_matrix(correction[3:])
                T[:3, 3] = correction[:3]
                local = quaternion_matrix(snapshot[stamp][3:])
                local[:3, 3] = snapshot[stamp][:3]
                expected = T.dot(local)
                actual = quaternion_matrix(pose[3:])
                actual[:3, 3] = pose[:3]
                if np.max(np.abs(expected - actual)) > 1e-6:
                    errors.append("map-to-odom global pose composition mismatch")
                global_checks += 1
            if args.faults:
                if not any(row["values"].get("gnss_live") == "0" and
                           row["values"].get("alignment_ready") == "1" for row in online_status):
                    errors.append("online GNSS loss not reported after alignment")
                if not online_status or online_status[-1]["values"].get("gnss_live") != "1":
                    errors.append("online GNSS did not recover")
                start = proxy_state["start"]
                if any(start + 28 <= t < start + 30 for t in factor_times):
                    errors.append("factors fabricated during GNSS dropout")
            if online_events and int(online_events[-1]["gnss_factors"]) != len(factor_times):
                errors.append("online factor audit count mismatch")
            if any(int(row["values"].get("queue_dropped", "0")) for row in online_status):
                errors.append("online event queue overflow")
            result["online"] = {"global_poses": len(global_poses), "global_checks": global_checks,
                                "global_maps": len(global_maps), "gnss_factors": len(factor_times),
                                "nodes": int(online_events[-1]["nodes"]) if online_events else 0,
                                "last_status": online_status[-1] if online_status else None}
            result["errors"] = sorted(set(errors))
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "poses"}, indent=2))
        print("Results:", output)
        if errors:
            raise RuntimeError("Data contract verification failed")
    finally:
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
