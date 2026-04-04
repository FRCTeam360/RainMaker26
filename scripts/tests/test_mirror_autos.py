"""Tests for mirror_autos.py — focused on rotation correctness."""

import sys
import unittest
from pathlib import Path

# Add scripts/ to the import path so we can import from the parent directory
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mirror_autos import (
    wrap_rotation,
    mirror_rotation,
    mirror_point,
    mirror_path_data,
    make_mirrored_name,
    strip_master,
)

FIELD_WIDTH = 8.07


class TestWrapRotation(unittest.TestCase):
    def test_no_wrap_needed(self):
        self.assertEqual(wrap_rotation(0), 0)
        self.assertEqual(wrap_rotation(90), 90)
        self.assertEqual(wrap_rotation(-90), -90)

    def test_wrap_positive_over_180(self):
        self.assertEqual(wrap_rotation(270), -90)

    def test_wrap_negative_under_neg180(self):
        self.assertEqual(wrap_rotation(-270), 90)

    def test_boundary_180(self):
        self.assertEqual(wrap_rotation(180), 180)

    def test_boundary_neg180(self):
        self.assertEqual(wrap_rotation(-180), 180)


class TestMirrorRotation(unittest.TestCase):
    """Mirror negates the angle (then wraps)."""

    def test_zero_stays_zero(self):
        self.assertEqual(mirror_rotation(0), 0)

    def test_90_becomes_neg90(self):
        self.assertEqual(mirror_rotation(90), -90)

    def test_neg90_becomes_90(self):
        self.assertEqual(mirror_rotation(-90), 90)

    def test_180_becomes_neg180_wraps_to_180(self):
        self.assertEqual(mirror_rotation(180), 180)

    def test_45_becomes_neg45(self):
        self.assertEqual(mirror_rotation(45), -45)

    def test_neg45_becomes_45(self):
        self.assertEqual(mirror_rotation(-45), 45)

    def test_1_becomes_neg1(self):
        self.assertEqual(mirror_rotation(1), -1)

    def test_neg179_becomes_179(self):
        self.assertEqual(mirror_rotation(-179), 179)

    def test_double_mirror_is_identity(self):
        """Mirroring twice should return the original rotation."""
        for angle in [0, 45, 90, -90, 135, -135, 180, -179, 1, -1, 179]:
            self.assertAlmostEqual(mirror_rotation(mirror_rotation(angle)), angle, places=3)


class TestMirrorPoint(unittest.TestCase):
    def test_centerline_stays(self):
        x, y = 5.0, FIELD_WIDTH / 2
        mx, my = mirror_point(x, y, FIELD_WIDTH)
        self.assertAlmostEqual(mx, x, places=4)
        self.assertAlmostEqual(my, y, places=4)

    def test_x_unchanged(self):
        x, y = 3.5, 2.0
        mx, my = mirror_point(x, y, FIELD_WIDTH)
        self.assertAlmostEqual(mx, x, places=4)

    def test_y_reflected(self):
        x, y = 3.5, 2.0
        mx, my = mirror_point(x, y, FIELD_WIDTH)
        self.assertAlmostEqual(my, FIELD_WIDTH - y, places=4)

    def test_double_mirror_is_identity(self):
        x, y = 3.5, 2.1
        mx, my = mirror_point(x, y, FIELD_WIDTH)
        mx2, my2 = mirror_point(mx, my, FIELD_WIDTH)
        self.assertAlmostEqual(mx2, x, places=4)
        self.assertAlmostEqual(my2, y, places=4)


class TestMirrorPathData(unittest.TestCase):
    """Test that mirror_path_data correctly transforms all rotation fields."""

    def _make_path(self, goal_rot, start_rot, rot_targets):
        return {
            "waypoints": [],
            "rotationTargets": [{"rotationDegrees": r} for r in rot_targets],
            "pointTowardsZones": [],
            "goalEndState": {"rotation": goal_rot},
            "idealStartingState": {"rotation": start_rot, "velocity": 0},
            "folder": None,
        }

    def test_rotations_mirrored(self):
        path = self._make_path(
            goal_rot=45, start_rot=-30, rot_targets=[90, -90, 0]
        )
        mirrored = mirror_path_data(path, FIELD_WIDTH)

        self.assertEqual(mirrored["goalEndState"]["rotation"], -45)
        self.assertEqual(mirrored["idealStartingState"]["rotation"], 30)
        self.assertEqual(mirrored["rotationTargets"][0]["rotationDegrees"], -90)
        self.assertEqual(mirrored["rotationTargets"][1]["rotationDegrees"], 90)
        self.assertEqual(mirrored["rotationTargets"][2]["rotationDegrees"], 0)

    def test_double_mirror_identity(self):
        path = self._make_path(
            goal_rot=-79.16, start_rot=180, rot_targets=[30, -120]
        )
        mirrored_once = mirror_path_data(path, FIELD_WIDTH)
        mirrored_twice = mirror_path_data(mirrored_once, FIELD_WIDTH)

        self.assertAlmostEqual(
            mirrored_twice["goalEndState"]["rotation"],
            path["goalEndState"]["rotation"],
            places=3,
        )
        self.assertAlmostEqual(
            mirrored_twice["idealStartingState"]["rotation"],
            path["idealStartingState"]["rotation"],
            places=3,
        )
        for orig, double in zip(
            path["rotationTargets"], mirrored_twice["rotationTargets"]
        ):
            self.assertAlmostEqual(
                double["rotationDegrees"], orig["rotationDegrees"], places=3
            )


class TestStripMaster(unittest.TestCase):
    def test_removes_master(self):
        self.assertEqual(strip_master("Master Left Blue"), "Left Blue")

    def test_case_insensitive(self):
        self.assertEqual(strip_master("MASTER Right Red"), "Right Red")

    def test_no_master(self):
        self.assertEqual(strip_master("Right Blue Middle"), "Right Blue Middle")


class TestMakeMirroredName(unittest.TestCase):
    def test_side_swap(self):
        self.assertEqual(make_mirrored_name("Right Blue"), "MIRRORED Left Blue")

    def test_master_stripped(self):
        self.assertEqual(
            make_mirrored_name("Master Right Blue"), "MIRRORED Left Blue"
        )


if __name__ == "__main__":
    unittest.main()
