"""Audit exported Hesai cloud timing/rings and IMU timestamps offline."""
import argparse
import bisect
import json
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def records(path):
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def audit(clouds, imu_stamps, rings=16, max_span=0.2, max_imu_gap=0.02):
    if type(rings) is not int or rings <= 0 or not finite(max_span) or max_span <= 0 or not finite(max_imu_gap) or max_imu_gap <= 0:
        raise ValueError("positive ring count and timing thresholds required")
    if any(not finite(t) for t in imu_stamps):
        raise ValueError("IMU timestamps must be finite seconds")
    if any(b <= a for a, b in zip(imu_stamps, imu_stamps[1:])):
        raise ValueError("IMU timestamps must be strictly increasing")
    errors, count, points_total, previous = [], 0, 0, None
    worst_span, worst_gap = 0.0, 0.0
    for count, frame in enumerate(clouds, 1):
        issues = []
        if not isinstance(frame, dict):
            raise ValueError("cloud records must be objects")
        stamp = frame.get("stamp")
        if not finite(stamp):
            errors.append({"frame": count, "reasons": ["invalid_header_stamp"]})
            continue
        if previous is not None and stamp <= previous:
            issues.append("nonincreasing_frame_stamp")
        previous = stamp
        fields = frame.get("fields", {})
        if not isinstance(fields, dict) or fields.get("ring") != "uint16" or fields.get("timestamp") != "float64":
            issues.append("ring_or_timestamp_schema")
        points = frame.get("points")
        if not isinstance(points, list) or not points:
            issues.append("empty_or_invalid_points")
            points = []
        times = []
        for point in points:
            if not isinstance(point, list) or len(point) != 2:
                issues.append("invalid_point_record")
                continue
            ring, time = point
            if type(ring) is not int or not 0 <= ring < rings:
                issues.append("ring_out_of_range")
            if not finite(time):
                issues.append("invalid_point_timestamp")
            else:
                times.append(time)
        points_total += len(points)
        if times:
            span = max(times) - min(times)
            worst_span = max(worst_span, span)
            if span > max_span:
                issues.append("frame_span_exceeded")
            if abs(min(times) - stamp) > max_span:
                issues.append("header_point_time_mismatch")
            # Check both ends of the point-time interval against nearby IMU samples.
            for time in (min(times), max(times)):
                index = bisect.bisect_left(imu_stamps, time)
                neighbours = imu_stamps[max(0, index-1):index+1]
                if not neighbours:
                    issues.append("no_imu_samples")
                else:
                    gap = min(abs(t-time) for t in neighbours)
                    worst_gap = max(worst_gap, gap)
                    if gap > max_imu_gap:
                        issues.append("imu_time_gap_exceeded")
        if issues:
            errors.append({"frame": count, "reasons": sorted(set(issues))})
    if not count:
        errors.append({"frame": None, "reasons": ["no_cloud_frames"]})
    return dict(schema=1, passed=not errors, cloud_frames=count, exported_points=points_total,
                maximum_frame_span_s=worst_span, maximum_endpoint_imu_gap_s=worst_gap,
                errors=errors, scope="exported_fields_and_timestamps")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clouds")
    parser.add_argument("--imu", required=True)
    parser.add_argument("--rings", type=int, default=16)
    parser.add_argument("--max-span", type=float, default=0.2)
    parser.add_argument("--max-imu-gap", type=float, default=0.02)
    args = parser.parse_args()
    try:
        result = audit(records(args.clouds), [r["stamp"] for r in records(args.imu)],
                       args.rings, args.max_span, args.max_imu_gap)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(json.dumps({"passed": False, "error": str(error)}))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
