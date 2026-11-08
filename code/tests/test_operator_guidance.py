"""Only synthetic calibration, images and sensor samples; no physical claims."""

from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from contour_tracking import Selection, find_regions
from operator_guidance import Calibration, Correspondence, guidance, load_calibration
from sensor_telemetry import parse_line


CALIBRATION = Calibration(640, 480, (300, 200), (10, 15), 1, 1, True, (8, 12))
SHAPE = (480, 640, 3)


def reading(distance=10, now=1):
    return parse_line(f"1000,580,OK,{distance},0,0,0,0,0", now)


class GuidanceTests(unittest.TestCase):
    def test_measured_crosshair_signs_and_deadband(self):
        for center, expected in (((320, 200), "move right"), ((280, 200), "move left"),
                                 ((300, 220), "move down"), ((300, 180), "move up"),
                                 ((320, 180), "move right / move up")):
            self.assertEqual(guidance(CALIBRATION, SHAPE, center).alignment, expected)
        for center in ((300, 200), (310, 215), (290, 185)):
            self.assertIn("Aligned", guidance(CALIBRATION, SHAPE, center).alignment)
        mirrored = replace(CALIBRATION, right_sign=-1, down_sign=-1)
        self.assertEqual(guidance(mirrored, SHAPE, (320, 220)).alignment, "move left / move up")

    def test_unset_unverified_and_changed_resolution_unavailable(self):
        for calibration, shape in ((None, SHAPE), (replace(CALIBRATION, axes_verified=False), SHAPE),
                                   (CALIBRATION, (720, 1280, 3))):
            result = guidance(calibration, shape, (300, 200), reading(), 1, True)
            self.assertIn("unavailable", result.alignment)
            self.assertIn("unavailable", result.range_advice)

    def test_range_requires_target_correspondence_window_and_fresh_valid_sample(self):
        for r, now in ((None, 1), (reading(), 2.1), (reading(now=2), 1),
                       (parse_line("1000,0,TIMEOUT,NA,0,0,0,0,0", 1), 1),
                       (parse_line("1000,29000,OUT_OF_RANGE,NA,0,0,0,0,0", 1), 1)):
            self.assertIn("unavailable", guidance(CALIBRATION, SHAPE, (300, 200), r, now, True).range_advice)
        self.assertIn("correspondence", guidance(CALIBRATION, SHAPE, (300, 200), reading(), 1).range_advice)
        self.assertIn("window unset", guidance(replace(CALIBRATION, range_window_cm=None), SHAPE,
                                               (300, 200), reading(), 1, True).range_advice)

    def test_measured_range_signs_and_inclusive_window(self):
        for distance, expected in ((7, "Move back"), (13, "Move closer"), (8, "Distance OK"),
                                   (10, "Distance OK"), (12, "Distance OK")):
            result = guidance(CALIBRATION, SHAPE, (300, 200), reading(distance), 1, True)
            self.assertIn(expected, result.range_advice)
            self.assertNotIn("ready", result.range_advice.lower())

    def test_lost_or_unknown_target_overrides_otherwise_valid_range(self):
        for center in (None, (float("nan"), 200), (float("inf"), 200), (-1, 200)):
            result = guidance(CALIBRATION, SHAPE, center, reading(), 1, True)
            self.assertIn("unavailable", result.alignment)
            self.assertIn("unavailable", result.range_advice)

    def test_calibration_loader_and_invalid_measurements_fail_closed(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.json"
            data = dict(width=640, height=480, crosshair=[300, 200], tolerance=[10, 15],
                        right_sign=1, down_sign=1, axes_verified=True, range_window_cm=[8, 12])
            path.write_text(json.dumps(data))
            self.assertTrue(load_calibration(path).matches(SHAPE))
            for key, value in (("crosshair", [float("nan"), 200]), ("tolerance", [0, 10]),
                               ("right_sign", None), ("axes_verified", 1), ("range_window_cm", [12, 8])):
                invalid = data | {key: value}
                path.write_text(json.dumps(invalid))
                with self.assertRaises(ValueError):
                    load_calibration(path)
            with self.assertRaises(ValueError):
                load_calibration(Path(directory) / "missing.json")


class CorrespondenceTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros(SHAPE, np.uint8)
        self.frame[150:250, 250:350] = 255
        self.mask, self.regions = find_regions(self.frame)
        self.selection = Selection()
        self.selection.choose((300, 200), self.regions)
        self.evidence = Correspondence()

    def test_confirmation_is_explicit_and_expires_on_loss_and_reselection(self):
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 1))
        self.assertTrue(self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1))
        self.assertTrue(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 1.1))
        self.selection.update([])
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 1.1))
        self.selection.choose((300, 200), self.regions)
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 1.1))

    def test_stale_invalid_range_and_changed_scene_clear_confirmation(self):
        self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1)
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 2.1))
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(now=3), 3))
        self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1)
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(13), 1))
        self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1)
        moved = np.zeros(SHAPE, np.uint8)
        moved[150:250, 275:375] = 255
        self.selection.update(find_regions(moved)[1])
        self.assertFalse(self.evidence.valid(CALIBRATION, SHAPE, self.selection, reading(), 1))

    def test_cannot_confirm_raw_range_without_calibration_or_target(self):
        for c in (None, replace(CALIBRATION, range_window_cm=None), replace(CALIBRATION, axes_verified=False)):
            self.assertFalse(self.evidence.confirm(c, SHAPE, self.selection, reading(), 1))
        self.selection.clear()
        self.assertFalse(self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1))

    def test_write_boundary_rechecks_confirmed_baseline_without_mutating_it(self):
        calibration = replace(CALIBRATION, range_window_cm=(8, 12))
        self.assertTrue(self.evidence.confirm(calibration, SHAPE, self.selection, reading(8), 1))
        self.assertTrue(self.evidence.valid(calibration, SHAPE, self.selection, reading(9.9), 1))
        self.assertFalse(self.evidence.matches(calibration, SHAPE, self.selection, reading(10.1), 1))
        self.assertEqual(self.evidence.distance, 8)
        self.assertIsNotNone(self.evidence.target)
        self.assertFalse(self.evidence.valid(calibration, SHAPE, self.selection, reading(10.1), 1))
        self.assertIsNone(self.evidence.target)

    def test_removed_range_calibration_invalidates_existing_confirmation(self):
        self.evidence.confirm(CALIBRATION, SHAPE, self.selection, reading(), 1)
        self.assertFalse(self.evidence.valid(replace(CALIBRATION, range_window_cm=None),
                                             SHAPE, self.selection, reading(), 1))


if __name__ == "__main__":
    unittest.main()
