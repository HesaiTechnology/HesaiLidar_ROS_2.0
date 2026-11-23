from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from slam_input_audit import audit


def cloud(stamp=10.0):
    return dict(stamp=stamp, fields=dict(ring="uint16", timestamp="float64"),
                points=[[0, stamp], [15, stamp+0.1]])


class AuditTests(unittest.TestCase):
    def reasons(self, frame, imu=None):
        return audit([frame], [10, 10.1] if imu is None else imu)["errors"][0]["reasons"]

    def test_valid_frame(self):
        self.assertTrue(audit([cloud()], [10, 10.1])["passed"])

    def test_schema_mismatch(self):
        frame = cloud()
        frame["fields"]["timestamp"] = "float32"
        self.assertIn("ring_or_timestamp_schema", self.reasons(frame))

    def test_ring_range(self):
        frame = cloud()
        frame["points"][0][0] = 16
        self.assertIn("ring_out_of_range", self.reasons(frame))

    def test_frame_span(self):
        frame = cloud()
        frame["points"][1][1] = 11
        self.assertIn("frame_span_exceeded", self.reasons(frame))

    def test_stamp_regression(self):
        result = audit([cloud(10), cloud(9)], [9, 9.1, 10, 10.1])
        self.assertIn("nonincreasing_frame_stamp", result["errors"][0]["reasons"])

    def test_point_time_is_not_required_to_follow_array_order(self):
        frame = cloud()
        frame["points"].reverse()
        self.assertTrue(audit([frame], [10, 10.1])["passed"])

    def test_imu_gap(self):
        self.assertIn("imu_time_gap_exceeded", self.reasons(cloud(), [20]))

    def test_missing_imu(self):
        self.assertIn("no_imu_samples", self.reasons(cloud(), []))

    def test_nonfinite_point_time(self):
        frame = cloud()
        frame["points"][0][1] = float("nan")
        self.assertIn("invalid_point_timestamp", self.reasons(frame))

    def test_imu_order_rejected(self):
        with self.assertRaises(ValueError):
            audit([cloud()], [10, 9])

    def test_empty_cloud_stream(self):
        self.assertFalse(audit([], [10])["passed"])

    def test_header_and_point_time_domains(self):
        frame = cloud()
        frame["stamp"] = 100
        self.assertIn("header_point_time_mismatch", self.reasons(frame))


if __name__ == "__main__":
    unittest.main()
