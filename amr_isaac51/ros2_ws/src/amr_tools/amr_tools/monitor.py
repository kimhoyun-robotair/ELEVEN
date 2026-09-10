"""Print stream health without requiring a graphical desktop."""

import argparse
import json
import math
import time

import rclpy
from rclpy.utilities import remove_ros_args

from .streams import StreamCollector


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float, default=5.0, help="wall seconds between reports")
    parser.add_argument("--json", action="store_true", help="emit one JSON object per report")
    parsed = parser.parse_args(remove_ros_args(args=args)[1:])
    if not math.isfinite(parsed.interval) or parsed.interval <= 0:
        parser.error("--interval must be positive")
    rclpy.init(args=args)
    node = StreamCollector()
    next_report = time.monotonic() + parsed.interval
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() < next_report:
                continue
            report = node.snapshot()
            if parsed.json:
                print(json.dumps({"topics": report, "problems": node.problems()}), flush=True)
            else:
                print(f"\n{'Topic':48s} {'wall Hz':>8s} {'count':>7s} {'age s':>6s}  status", flush=True)
                for topic, item in report.items():
                    age = "--" if item["age_wall_s"] is None else f"{item['age_wall_s']:.1f}"
                    status = "; ".join(item["errors"]) or "OK"
                    print(f"{topic:48s} {item['wall_hz']:8.2f} {item['count']:7d} {age:>6s}  {status}")
                print("Rates are wall-clock throughput; real-time factor changes these rates.", flush=True)
                tf_errors = [p for p in node.problems() if p.startswith("TF missing:")]
                for issue in tf_errors:
                    print(issue, flush=True)
            next_report = time.monotonic() + parsed.interval
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
