"""Synthetic image and identity-loss checks; no physical camera is used."""

import unittest

import cv2
import numpy as np

from contour_tracking import Region, Selection, find_regions


def region(box, pixels=None):
    x, y, w, h = box
    return Region(box, pixels or w * h, (x + w / 2, y + h / 2), np.empty((0, 1, 2), dtype=np.int32))


class ContourTests(unittest.TestCase):
    def test_separated_regions_and_noise_filter(self):
        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[20:40, 30:60] = 255
        frame[60:80, 100:120] = 255
        frame[5, 5] = 255
        mask, regions = find_regions(frame, min_pixels=10)
        self.assertEqual([r.box for r in regions], [(30, 20, 30, 20), (100, 60, 20, 20)])
        self.assertEqual(regions[0].pixels, 600)
        self.assertEqual(mask[5, 5], 255)  # Show the real mask, including filtered noise.

    def test_roi_uses_full_image_coordinates(self):
        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[20:40, 30:60] = 255
        frame[60:80, 100:120] = 255
        mask, regions = find_regions(frame, min_pixels=10, roi=(90, 50, 50, 40))
        self.assertEqual([r.box for r in regions], [(100, 60, 20, 20)])
        self.assertEqual(mask[25, 35], 0)
        self.assertEqual(regions[0].center, (109.5, 69.5))

    def test_thin_wire_is_preserved_by_pixel_area(self):
        frame = np.zeros((100, 160, 3), dtype=np.uint8)
        frame[50, 20:120] = 255
        _, regions = find_regions(frame, min_pixels=20)
        self.assertEqual(len(regions), 1)
        self.assertEqual(regions[0].pixels, 100)
        self.assertEqual(regions[0].box, (20, 50, 100, 1))

    def test_loop_hole_is_not_filled(self):
        frame = np.zeros((120, 120, 3), dtype=np.uint8)
        cv2.circle(frame, (60, 60), 30, (255, 255, 255), 2)
        mask, regions = find_regions(frame, min_pixels=20)
        self.assertEqual(len(regions), 1)
        self.assertEqual(mask[60, 60], 0)

    def test_dark_object_on_light_background(self):
        frame = np.full((100, 160, 3), 255, dtype=np.uint8)
        frame[20:40, 30:60] = 0
        _, regions = find_regions(frame, dark=True)
        self.assertEqual([r.box for r in regions], [(30, 20, 30, 20)])

    def test_uniform_foreground_does_not_box_whole_view(self):
        frame = np.full((100, 160, 3), 255, dtype=np.uint8)
        _, regions = find_regions(frame)
        self.assertEqual(regions, [])


class SelectionTests(unittest.TestCase):
    def test_selection_follows_one_nearby_region(self):
        selected = region((20, 20, 30, 30))
        moved = region((25, 24, 30, 30))
        selection = Selection()
        selection.choose((25, 25), [selected, region((100, 100, 30, 30))])
        selection.update([moved, region((100, 100, 30, 30))])
        self.assertEqual(selection.state, "tracking")
        self.assertIs(selection.current, moved)
        self.assertEqual(selection.number, 1)

    def test_loss_does_not_reacquire_or_switch_to_neighbor(self):
        original = region((20, 20, 30, 30))
        selection = Selection()
        selection.choose((25, 25), [original])
        selection.update([region((100, 100, 30, 30))])
        self.assertEqual(selection.state, "lost")
        selection.update([original])
        self.assertEqual(selection.state, "lost")
        self.assertIsNone(selection.current)
        selection.choose((25, 25), [original])
        self.assertEqual(selection.state, "tracking")

    def test_merged_region_is_not_accepted_as_same_target(self):
        original = region((20, 20, 30, 30))
        selection = Selection()
        selection.choose((25, 25), [original])
        selection.update([region((20, 20, 60, 30))])
        self.assertEqual(selection.state, "lost")
        self.assertIsNone(selection.current)

    def test_two_plausible_matches_require_new_selection(self):
        original = region((40, 40, 40, 40), pixels=600)
        selection = Selection()
        selection.choose((45, 45), [original])
        selection.update([
            region((35, 40, 35, 40), pixels=600),
            region((65, 40, 35, 40), pixels=600),
        ])
        self.assertEqual(selection.state, "ambiguous")
        self.assertIsNone(selection.current)

    def test_overlapping_click_does_not_silently_choose(self):
        selection = Selection()
        selection.choose((50, 50), [region((20, 20, 60, 60)), region((40, 40, 20, 20))])
        self.assertEqual(selection.state, "ambiguous")
        self.assertIsNone(selection.current)

    def test_guided_motion_allows_bounded_displacement_without_overlap(self):
        original = region((100, 70, 30, 100))
        moved = region((190, 70, 30, 100))
        selection = Selection()
        selection.choose((115, 100), [original])
        selection.update([moved], moving=True)
        self.assertEqual(selection.state, "tracking")
        self.assertIs(selection.current, moved)

    def test_guided_motion_rejects_same_area_with_a_different_shape(self):
        selection = Selection()
        selection.choose((115, 100), [region((100, 70, 30, 100))])
        selection.update([region((190, 20, 15, 200))], moving=True)
        self.assertEqual(selection.state, "lost")
        self.assertIsNone(selection.current)


if __name__ == "__main__":
    unittest.main()
