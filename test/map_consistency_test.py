#!/usr/bin/env python3
import tempfile
import unittest
from pathlib import Path
import numpy as np
from map_consistency import metrics, read_pcd


class MapConsistencyTests(unittest.TestCase):
    def test_plane_thickness_and_reference_retention(self):
        points = np.array([[x, y, z] for x in (0, 1, 2) for y in (0, 1, 2) for z in (-0.01, 0.01)])
        labels = {"static_planes": [{"name": "wall", "min": [-1, -1, -1], "max": [3, 3, 1], "normal": [0, 0, 1]}]}
        row = metrics(points, labels, points)["static_planes"][0]
        self.assertAlmostEqual(row["plane_rms_m"], 0.01)
        self.assertAlmostEqual(row["normal_error_rad"], 0)
        self.assertEqual(row["reference_voxel_retention"], 1)

    def test_empty_and_degenerate_roi_is_not_perfect_plane(self):
        points = np.array([[x, 0, 0] for x in (0, 1, 2)])
        labels = {"static_planes": [{"name": "line", "min": [-1, -1, -1], "max": [3, 3, 1]}]}
        self.assertIsNone(metrics(points, labels)["static_planes"][0]["plane_rms_m"])

    def test_dynamic_roi_count_is_not_a_semantic_detection(self):
        points = np.array([[0, 0, 0], [5, 5, 5]], float)
        labels = {"dynamic_regions": [{"name": "empty_zone", "min": [-1, -1, -1], "max": [1, 1, 1], "expected_empty": True}]}
        self.assertEqual(metrics(points, labels)["dynamic_regions"][0]["map_fraction"], 0.5)

    def test_binary_pcd_xyz(self):
        with tempfile.TemporaryDirectory(prefix="livo-map-test-") as directory:
            path = Path(directory) / "map.pcd"
            header = b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"
            path.write_bytes(header + np.array([[1, 2, 3], [4, 5, 6]], dtype="<f4").tobytes())
            self.assertTrue(np.allclose(read_pcd(path), [[1, 2, 3], [4, 5, 6]]))

    def test_empty_map_has_no_false_plane_quality(self):
        with tempfile.TemporaryDirectory(prefix="livo-map-test-") as directory:
            path = Path(directory) / "empty.pcd"
            path.write_text("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA ascii\n")
            self.assertEqual(read_pcd(path).shape, (0, 3))


if __name__ == "__main__":
    unittest.main()
