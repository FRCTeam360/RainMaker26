"""Tests for flip_autos.py — focused on rotation correctness."""

import sys
import unittest
from pathlib import Path

# Add scripts/ to the import path so we can import from the parent directory
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flip_autos import (
    wrap_rotation,
    flip_rotation,
    flip_point,
    flip_path_data,
    make_flipped_name,
    strip_master,
)

FIELD_LENGTH = 16.54
FIELD_WIDTH = 8.07


class TestWrapRotation(unittest.TestCase):
    def test_no_wrap_needed(self):
        self.assertEqual(wrap_rotation(0), 0)
        self.assertEqual(wrap_rotation(90), 90)
        self.assertEqual(wrap_rotation(-90), -90)

    def test_wrap_positive_over_180(self):
        self.assertEqual(wrap_rotation(270), -90)
        self.assertEqual(wrap_rotation(360), 0)

    def test_wrap_negative_under_neg180(self):
        self.assertEqual(wrap_rotation(-270), 90)
        self.assertEqual(wrap_rotation(-360), 0)

    def test_boundary_180(self):
        self.assertEqual(wrap_rotation(180), 180)

    def test_boundary_neg180(self):
        # -180 should wrap to 180
        self.assertEqual(wrap_rotation(-180), 180)

    def test_large_values(self):
        self.assertEqual(wrap_rotation(720), 0)
        self.assertEqual(wrap_rotation(-720), 0)


class TestFlipRotation(unittest.TestCase):
    """Flip adds 180 degrees (then wraps)."""

    def test_zero_becomes_180(self):
        self.assertEqual(flip_rotation(0), 180)

    def test_180_becomes_zero(self):
        self.assertEqual(flip_rotation(180), 0)

    def test_90_becomes_neg90(self):
        self.assertEqual(flip_rotation(90), -90)

    def test_neg90_becomes_90(self):
        self.assertEqual(flip_rotation(-90), 90)

    def test_45_becomes_neg135(self):
        self.assertEqual(flip_rotation(45), -135)

    def test_neg45_becomes_135(self):
        self.assertEqual(flip_rotation(-45), 135)

    def test_neg180_becomes_zero(self):
        self.assertEqual(flip_rotation(-180), 0)

    def test_1_becomes_neg179(self):
        self.assertEqual(flip_rotation(1), -179)

    def test_neg1_becomes_179(self):
        self.assertEqual(flip_rotation(-1), 179)

    def test_double_flip_is_identity(self):
        """Flipping twice should return the original rotation."""
        for angle in [0, 45, 90, -90, 135, -135, 180, -179, 1, -1, 179]:
            self.assertAlmostEqual(flip_rotation(flip_rotation(angle)), angle, places=3)


class TestFlipPoint(unittest.TestCase):
    def test_center_stays_at_center(self):
        cx, cy = FIELD_LENGTH / 2, FIELD_WIDTH / 2
        fx, fy = flip_point(cx, cy, FIELD_LENGTH, FIELD_WIDTH)
        self.assertAlmostEqual(fx, cx, places=4)
        self.assertAlmostEqual(fy, cy, places=4)

    def test_origin_flips_to_far_corner(self):
        fx, fy = flip_point(0, 0, FIELD_LENGTH, FIELD_WIDTH)
        self.assertAlmostEqual(fx, FIELD_LENGTH, places=4)
        self.assertAlmostEqual(fy, FIELD_WIDTH, places=4)

    def test_double_flip_is_identity(self):
        x, y = 3.5, 2.1
        fx, fy = flip_point(x, y, FIELD_LENGTH, FIELD_WIDTH)
        fx2, fy2 = flip_point(fx, fy, FIELD_LENGTH, FIELD_WIDTH)
        self.assertAlmostEqual(fx2, x, places=4)
        self.assertAlmostEqual(fy2, y, places=4)


class TestFlipPathData(unittest.TestCase):
    """Test that flip_path_data correctly transforms all rotation fields."""

    def _make_path(self, goal_rot, start_rot, rot_targets):
        return {
            "waypoints": [],
            "rotationTargets": [{"rotationDegrees": r} for r in rot_targets],
            "pointTowardsZones": [],
            "goalEndState": {"rotation": goal_rot},
            "idealStartingState": {"rotation": start_rot, "velocity": 0},
            "folder": None,
        }

    def test_rotations_flipped(self):
        path = self._make_path(
            goal_rot=0, start_rot=90, rot_targets=[45, -45, 180]
        )
        flipped = flip_path_data(path, FIELD_LENGTH, FIELD_WIDTH)

        self.assertEqual(flipped["goalEndState"]["rotation"], 180)
        self.assertEqual(flipped["idealStartingState"]["rotation"], -90)
        self.assertEqual(flipped["rotationTargets"][0]["rotationDegrees"], -135)
        self.assertEqual(flipped["rotationTargets"][1]["rotationDegrees"], 135)
        self.assertEqual(flipped["rotationTargets"][2]["rotationDegrees"], 0)

    def test_double_flip_identity(self):
        path = self._make_path(
            goal_rot=-79.16, start_rot=180, rot_targets=[30, -120]
        )
        flipped_once = flip_path_data(path, FIELD_LENGTH, FIELD_WIDTH)
        flipped_twice = flip_path_data(flipped_once, FIELD_LENGTH, FIELD_WIDTH)

        self.assertAlmostEqual(
            flipped_twice["goalEndState"]["rotation"],
            path["goalEndState"]["rotation"],
            places=3,
        )
        self.assertAlmostEqual(
            flipped_twice["idealStartingState"]["rotation"],
            path["idealStartingState"]["rotation"],
            places=3,
        )
        for orig, double in zip(
            path["rotationTargets"], flipped_twice["rotationTargets"]
        ):
            self.assertAlmostEqual(
                double["rotationDegrees"], orig["rotationDegrees"], places=3
            )


class TestStripMaster(unittest.TestCase):
    def test_removes_master(self):
        self.assertEqual(strip_master("Master Red Right"), "Red Right")

    def test_case_insensitive(self):
        self.assertEqual(strip_master("MASTER Blue Left"), "Blue Left")
        self.assertEqual(strip_master("master Blue Left"), "Blue Left")

    def test_mid_string(self):
        self.assertEqual(strip_master("Red Master Middle"), "Red Middle")

    def test_no_master(self):
        self.assertEqual(strip_master("Red Right Middle"), "Red Right Middle")


class TestMakeFlippedName(unittest.TestCase):
    def test_alliance_swap(self):
        self.assertEqual(make_flipped_name("Red Right"), "FLIPPED Blue Right")

    def test_master_stripped(self):
        self.assertEqual(
            make_flipped_name("Master Red Right"), "FLIPPED Blue Right"
        )


if __name__ == "__main__":
    unittest.main()
