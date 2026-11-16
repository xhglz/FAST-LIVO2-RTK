#!/usr/bin/env python3
import unittest
from resource_report import distribution, memory_metrics, validity_metrics, paired_processing


class ResourceReportTests(unittest.TestCase):
    def test_percentiles_and_empty_samples(self):
        self.assertEqual(distribution([])["p99"], None)
        self.assertAlmostEqual(distribution(range(101))["p95"], 95)
        self.assertAlmostEqual(distribution(range(101))["p99"], 99)

    def test_nonfinite_samples_are_not_statistics(self):
        self.assertEqual(distribution([float("nan"), float("inf"), 2])["count"], 1)

    def test_pair_cost_uses_same_measurement_time(self):
        data = [{"time": "1", "stage": "LIO", "processing_ms": 3},
                {"time": "1", "stage": "VIO", "processing_ms": 4},
                {"time": "2", "stage": "LIO", "processing_ms": 100}]
        result = paired_processing(data)
        self.assertEqual(result["count"], 1)
        self.assertEqual(result["p99"], 7)

    def test_rss_warmup_and_slope(self):
        data = [{"sim_time": 100 + i, "wall_time": 2 * i, "rss_kb": 1024 * (20 + i)}
                for i in range(30)]
        metric = memory_metrics(data, warmup=10)
        self.assertEqual(metric["peak_rss_mb"], 49)
        self.assertEqual(metric["steady_growth_mb"], 19)
        self.assertAlmostEqual(metric["slope_mb_per_wall_min"], 30)
        self.assertAlmostEqual(metric["slope_mb_per_sim_min"], 60)

    def test_initialization_is_not_a_loss(self):
        data = [{"time": 1, "state": 0, "valid": False}, {"time": 2, "state": 1, "valid": True}]
        self.assertEqual(validity_metrics(data)["loss_episodes"], 0)

    def test_recovery_and_censored_interval(self):
        data = [{"time": 1, "state": 1, "valid": True},
                {"time": 2, "state": 3, "valid": False},
                {"time": 3, "state": 4, "valid": False},
                {"time": 4, "state": 1, "valid": True},
                {"time": 5, "state": 3, "valid": False},
                {"time": 6, "state": 3, "valid": False}]
        metric = validity_metrics(data)
        self.assertEqual(metric["loss_episodes"], 2)
        self.assertEqual(metric["recovery_seconds"], [2])
        self.assertTrue(metric["invalid_intervals"][-1]["censored"])
        self.assertAlmostEqual(metric["invalid_time_fraction"], 3 / 5)


if __name__ == "__main__":
    unittest.main()
