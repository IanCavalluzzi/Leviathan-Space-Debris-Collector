"""Color/shape regression scenes; these are not bench-camera measurements."""

import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from contour_tracking import Selection, find_regions
from guided_foreground import ForegroundGuide, choose_foreground


def bottle_scene(shift=0):
    image = np.full((150, 200, 3), 180, np.uint8)
    # Gray shadow touches a colored body; a bright cap exceeds a dark cutoff.
    image[65:125, 100 + shift:145 + shift] = 95
    image[40:120, 50 + shift:100 + shift] = (200, 50, 35)
    image[20:40, 60 + shift:90 + shift] = 230
    return image


def motion_scene(positions):
    image = np.full((240, 640, 3), 180, np.uint8)
    for x in positions:
        image[70:170, x:x + 30] = (200, 50, 35)
    return image


class GuidedForegroundTests(unittest.TestCase):
    def setUp(self):
        cv2.setRNGSeed(7)

    def test_whole_multicolor_bottle_excludes_touching_gray_shadow(self):
        frame = bottle_scene()
        original = frame.copy()
        brightness, _ = find_regions(frame, threshold=110, dark=True)
        self.assertEqual(brightness[25, 70], 0)  # Bright cap was missing.
        self.assertEqual(brightness[90, 130], 255)  # Shadow was included.
        guide = ForegroundGuide(frame, (45, 15, 60, 110))
        mask, regions = guide.detect(frame)
        self.assertEqual(mask[25, 70], 255)
        self.assertEqual(mask[90, 130], 0)
        self.assertEqual([r.box for r in regions], [(50, 20, 50, 100)])
        np.testing.assert_array_equal(frame, original)

    def test_slow_motion_uses_existing_selection_and_missing_target_is_lost(self):
        guide = ForegroundGuide(bottle_scene(), (45, 15, 60, 110))
        _, regions = guide.detect(bottle_scene())
        selected = Selection()
        selected.choose((70, 60), regions)
        _, moved = guide.detect(bottle_scene(8), selected.current.box)
        selected.update(moved)
        self.assertEqual(selected.state, "tracking")
        self.assertEqual(selected.current.box, (58, 20, 50, 100))
        empty = np.full((150, 200, 3), 180, np.uint8)
        _, absent = guide.detect(empty, selected.current.box)
        selected.update(absent)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)

    def test_corrections_can_mark_cap_and_remove_shadow(self):
        guide = ForegroundGuide(bottle_scene(), (45, 15, 60, 110))
        guide.mark((70, 25), True)
        guide.mark((130, 90), False)
        guide.refine()
        mask = guide.mask()
        self.assertEqual(mask[25, 70], 255)
        self.assertEqual(mask[90, 130], 0)

    def test_resized_camera_requires_new_outline(self):
        guide = ForegroundGuide(bottle_scene(), (45, 15, 60, 110))
        with self.assertRaisesRegex(ValueError, "size changed"):
            guide.detect(np.zeros((300, 400, 3), np.uint8))

    def test_invalid_and_all_frame_rectangles_are_rejected(self):
        for box in ((-1, 0, 20, 20), (0, 0, 1, 1), (190, 10, 20, 20), (0, 0, 200, 150)):
            with self.subTest(box=box), self.assertRaises(ValueError):
                ForegroundGuide(bottle_scene(), box)

    def moving_target(self, positions=(100,)):
        frame = motion_scene(positions)
        guide = ForegroundGuide(frame, (95, 65, 40, 110))
        _, regions = guide.detect(frame)
        selected = Selection()
        selected.choose((115, 100), regions)
        self.assertEqual(selected.state, "tracking")
        return guide, selected

    def test_narrow_target_can_move_without_overlapping_its_previous_box(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([135]), selected)
        self.assertEqual(selected.state, "tracking")
        self.assertEqual(selected.current.box, (135, 70, 30, 100))
        self.assertEqual(selected.number, 1)

    def test_target_beyond_nearby_crop_is_found_without_a_clipped_box(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([190]), selected)
        self.assertEqual(selected.state, "tracking")
        self.assertEqual(selected.current.box, (190, 70, 30, 100))

    def test_partial_nearby_foreground_expands_before_accepting_a_smaller_box(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([145]), selected)
        self.assertEqual(selected.state, "tracking")
        self.assertEqual(selected.current.box, (145, 70, 30, 100))

    def test_repeated_larger_moves_leave_the_initial_region_and_keep_fixed_colors(self):
        guide, selected = self.moving_target()
        foreground, background = guide.foreground.copy(), guide.background.copy()
        for x in (170, 240, 310, 380):
            guide.follow(motion_scene([x]), selected)
            self.assertEqual(selected.state, "tracking")
            self.assertEqual(selected.current.box, (x, 70, 30, 100))
        np.testing.assert_array_equal(guide.foreground, foreground)
        np.testing.assert_array_equal(guide.background, background)

    def test_two_new_plausible_targets_in_wider_search_are_ambiguous(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([30, 190]), selected)
        self.assertEqual(selected.state, "ambiguous")
        self.assertIsNone(selected.current)

    def test_disappearing_target_does_not_switch_to_a_stationary_lookalike(self):
        guide, selected = self.moving_target((100, 190))
        guide.follow(motion_scene([190]), selected)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)

    def test_wider_search_still_limits_displacement(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([300]), selected)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)

    def test_loss_still_requires_reselection_when_target_returns(self):
        guide, selected = self.moving_target()
        guide.follow(motion_scene([]), selected)
        self.assertEqual(selected.state, "lost")
        guide.follow(motion_scene([100]), selected, (100, 70, 30, 100))
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)


class ForegroundDialogTests(unittest.TestCase):
    def test_hd_correction_strokes_map_past_header_to_camera_pixels(self):
        frame = np.zeros((720, 1280, 3), np.uint8)
        guide = MagicMock()
        guide.mask.return_value = np.zeros((720, 1280), np.uint8)
        callbacks = []

        def input_key(delay):
            callbacks[0](cv2.EVENT_LBUTTONDOWN, 300, 252, 0, None)
            callbacks[0](cv2.EVENT_MOUSEMOVE, 320, 322, cv2.EVENT_FLAG_RBUTTON, None)
            return 13

        with patch("guided_foreground.cv2.selectROI", return_value=(250, 50, 100, 250)), \
                patch("guided_foreground.ForegroundGuide", return_value=guide) as make_guide, \
                patch("guided_foreground.cv2.setMouseCallback", side_effect=lambda name, cb: callbacks.append(cb)), \
                patch("guided_foreground.cv2.imshow"), \
                patch("guided_foreground.cv2.waitKey", side_effect=input_key), \
                patch("guided_foreground.cv2.destroyWindow") as close:
            self.assertIs(choose_foreground(frame), guide)
        np.testing.assert_array_equal(make_guide.call_args.args[0], frame)
        self.assertEqual(make_guide.call_args.args[1], (500, 100, 200, 500))
        guide.mark.assert_any_call((600, 400), True)
        guide.mark.assert_any_call((640, 540), False)
        guide.refine.assert_called_once()
        close.assert_called_once()

    def test_cancelled_outline_does_not_learn_a_model(self):
        with patch("guided_foreground.cv2.selectROI", return_value=(0, 0, 0, 0)), \
                patch("guided_foreground.ForegroundGuide") as make_guide, \
                patch("guided_foreground.cv2.destroyWindow") as close:
            self.assertIsNone(choose_foreground(np.zeros((720, 1280, 3), np.uint8)))
        make_guide.assert_not_called()
        close.assert_called_once()

    def test_cancelled_correction_closes_dialog_without_accepting(self):
        guide = MagicMock()
        guide.mask.return_value = np.zeros((720, 1280), np.uint8)
        with patch("guided_foreground.cv2.selectROI", return_value=(250, 50, 100, 250)), \
                patch("guided_foreground.ForegroundGuide", return_value=guide), \
                patch("guided_foreground.cv2.setMouseCallback"), \
                patch("guided_foreground.cv2.imshow"), \
                patch("guided_foreground.cv2.waitKey", return_value=27), \
                patch("guided_foreground.cv2.destroyWindow") as close:
            self.assertIsNone(choose_foreground(np.zeros((720, 1280, 3), np.uint8)))
        guide.refine.assert_not_called()
        close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
