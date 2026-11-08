"""Cloud contract, nonblocking requests, and conservative local handoff without hardware."""

import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import time
import unittest
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError, URLError

import cv2
import numpy as np

import gemini_selection as gs
from contour_tracking import overlap


def target_json(**changes):
    data = {"found": True, "label": "blue bottle", "box_2d": [200, 150, 600, 450],
            "reason": "Only blue object"}
    return json.dumps(data | changes)


def response_body(text, status="completed"):
    return json.dumps({"status": status, "steps": [
        {"type": "thought", "content": [{"type": "text", "text": "ignored"}]},
        {"type": "model_output", "content": [{"type": "text", "text": text}]},
    ]}).encode()


def fragmented_bottle_scene(shift=0, label=True):
    frame = np.full((240, 320, 3), 180, np.uint8)
    if label:
        frame[100:150, 80 + shift:130 + shift] = (200, 40, 30)
    # Disconnected reflections on a transparent neck, not additional objects.
    frame[70:80, 90 + shift:100 + shift] = 240
    frame[85:95, 110 + shift:120 + shift] = 240
    return frame


class GeminiSelectionTests(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((180, 240, 3), np.uint8)
        cv2.circle(self.frame, (72, 72), 25, (255, 80, 20), -1)
        cv2.circle(self.frame, (175, 75), 25, (0, 180, 255), -1)

    def test_normalized_box_uses_y_x_order_and_original_dimensions(self):
        target = gs.parse_target(target_json())
        self.assertEqual(target.pixel_box((720, 1280, 3)), (192, 144, 384, 288))
        edge = gs.parse_target(target_json(box_2d=[0, 0, 1000, 1000]))
        self.assertEqual(edge.pixel_box((181, 241, 3)), (0, 0, 241, 181))

    def test_malformed_or_unsafe_model_values_are_rejected(self):
        bad = ["not JSON", "[]", "{}", target_json(found="true"), target_json(found=1),
               target_json(label=""), target_json(label="x" * 121), target_json(reason=None),
               target_json(box_2d=[1, 2, 3]), target_json(box_2d=[0, 0, 1001, 500]),
               target_json(box_2d=[0, False, 500, 500]), target_json(box_2d=[0, 0, 1.5, 500]),
               target_json(box_2d=[600, 100, 200, 300]), target_json(box_2d=[0, 0, 0, 10]),
               target_json(motor_steps=1000)]
        for text in bad:
            with self.subTest(text=text), self.assertRaises(gs.GeminiError):
                gs.parse_target(text)

    def test_no_match_does_not_create_a_target(self):
        self.assertIsNone(gs.parse_target(target_json(found=False, label="", box_2d=[0, 0, 0, 0])))

    def test_https_request_sends_image_schema_key_header_and_disables_storage(self):
        response = MagicMock()
        response.__enter__.return_value.read.return_value = response_body(target_json())
        with patch.object(gs, "urlopen", return_value=response) as http:
            target = gs.select_target(self.frame, "the blue bottle", "test-secret", "test-model")
        self.assertEqual(target.label, "blue bottle")
        request = http.call_args.args[0]
        body = json.loads(request.data)
        self.assertEqual(request.full_url, gs.ENDPOINT)
        self.assertNotIn("test-secret", request.full_url)
        self.assertEqual(request.get_header("X-goog-api-key"), "test-secret")
        self.assertEqual(http.call_args.kwargs["timeout"], 20)
        self.assertEqual(body["model"], "test-model")
        self.assertEqual(body["input"][1]["type"], "image")
        self.assertTrue(body["input"][1]["data"])
        self.assertEqual(body["response_format"]["schema"], gs.SCHEMA)
        self.assertFalse(body["store"])
        self.assertNotIn("tools", body)

    def test_http_and_network_failures_have_safe_actionable_messages(self):
        for error, expected in ((HTTPError(gs.ENDPOINT, 429, "test-secret", {}, None), "Quota"),
                                (HTTPError(gs.ENDPOINT, 403, "test-secret", {}, None), "permissions"),
                                (URLError("test-secret"), "network/timeout"),
                                (TimeoutError("test-secret"), "network/timeout")):
            with self.subTest(error=error), patch.object(gs, "urlopen", side_effect=error):
                with self.assertRaises(gs.GeminiError) as caught:
                    gs.select_target(self.frame, "bottle", "test-secret", "model")
                self.assertIn(expected, str(caught.exception))
                self.assertNotIn("test-secret", str(caught.exception))

    def test_incomplete_or_malformed_api_response_cannot_select(self):
        for raw in (b"{}", b"not-json", response_body(target_json(), "in_progress"),
                    response_body(""), b"x" * 1_000_001):
            response = MagicMock()
            response.__enter__.return_value.read.return_value = raw
            with self.subTest(raw=raw[:40]), patch.object(gs, "urlopen", return_value=response):
                with self.assertRaises(gs.GeminiError):
                    gs.select_target(self.frame, "bottle", "secret", "model")

    def test_missing_credentials_never_attempt_network(self):
        with patch.dict(os.environ, {}, clear=True), patch.object(gs, "KEY_FILE") as key_file, \
                patch.object(gs, "urlopen") as http:
            key_file.read_text.side_effect = FileNotFoundError
            with self.assertRaisesRegex(gs.GeminiError, "GEMINI_API_KEY"):
                gs.GeminiSelector().submit(self.frame, "bottle")
        http.assert_not_called()

    def test_environment_credentials_take_precedence_over_local_file(self):
        for environment, expected in (
                ({"GEMINI_API_KEY": "session-key", "GOOGLE_API_KEY": "fallback-key"}, "session-key"),
                ({"GOOGLE_API_KEY": "fallback-key"}, "fallback-key")):
            with self.subTest(environment=list(environment)), \
                    patch.dict(os.environ, environment, clear=True), patch.object(gs, "KEY_FILE") as key_file:
                key_file.read_text.side_effect = PermissionError("must not read saved key")
                self.assertEqual(gs.get_api_key(), expected)
                key_file.read_text.assert_not_called()

    def test_saved_key_accepts_utf8_bom_and_surrounding_whitespace(self):
        with TemporaryDirectory() as directory:
            key_file = Path(directory) / "gemini_api_key.txt"
            key_file.write_text("\ufeff  saved-test-key \n", encoding="utf-8")
            with patch.dict(os.environ, {}, clear=True), patch.object(gs, "KEY_FILE", key_file):
                self.assertEqual(gs.get_api_key(), "saved-test-key")

    def test_missing_or_blank_saved_key_is_unconfigured(self):
        with TemporaryDirectory() as directory:
            for index, content in enumerate((None, "", "  \n")):
                key_file = Path(directory) / f"key-{index}.txt"
                if content is not None:
                    key_file.write_text(content, encoding="utf-8")
                with self.subTest(content=content), patch.dict(os.environ, {}, clear=True), \
                        patch.object(gs, "KEY_FILE", key_file):
                    self.assertEqual(gs.get_api_key(), "")

    def test_saved_key_read_errors_are_actionable_without_echoing_contents(self):
        for error in (PermissionError("test-secret"),
                      UnicodeDecodeError("utf-8", b"\xff", 0, 1, "test-secret")):
            with self.subTest(error=type(error).__name__), patch.dict(os.environ, {}, clear=True), \
                    patch.object(gs, "KEY_FILE") as key_file:
                key_file.read_text.side_effect = error
                with self.assertRaisesRegex(gs.GeminiError, "gemini_api_key.txt") as caught:
                    gs.get_api_key()
                self.assertNotIn("test-secret", str(caught.exception))

    def test_worker_is_nonblocking_single_request_and_cancellation_discards_late_result(self):
        started, release = Event(), Event()

        def blocked(*args):
            started.set()
            release.wait(2)
            return gs.parse_target(target_json())

        selector = gs.GeminiSelector()
        with patch.dict(os.environ, {"GEMINI_API_KEY": "test-secret"}), \
                patch.object(gs, "select_target", side_effect=blocked):
            try:
                selector.submit(self.frame, "bottle")
                self.assertTrue(started.wait(1))
                self.assertTrue(selector.busy)
                self.assertIsNone(selector.poll())
                with self.assertRaisesRegex(gs.GeminiError, "still running"):
                    selector.submit(self.frame, "another bottle")
                selector.cancel()
            finally:
                release.set()
                selector._thread.join(2)
        self.assertFalse(selector.busy)
        self.assertIsNone(selector.poll())

    def test_worker_copies_frame_and_returns_safe_failure(self):
        selector = gs.GeminiSelector()
        with patch.dict(os.environ, {"GEMINI_API_KEY": "test-secret"}), \
                patch.object(gs, "select_target", side_effect=RuntimeError("test-secret")):
            selector.submit(self.frame, "bottle")
            selector._thread.join(2)
        result = selector.poll()
        self.assertIsNot(result.frame, self.frame)
        np.testing.assert_array_equal(result.frame, self.frame)
        self.assertIsNone(result.target)
        self.assertNotIn("test-secret", result.error)

    def test_operator_confirmation_seeds_existing_local_tracker(self):
        proposal = gs.Proposal(self.frame.copy(), gs.parse_target(target_json()), time.monotonic())
        guide, selection = gs.confirm_target(proposal, self.frame)
        self.assertEqual(selection.state, "tracking")
        self.assertLess(selection.current.center[0], 120)
        self.assertIn("confirmed by operator", selection.reason)
        moved = np.zeros_like(self.frame)
        cv2.circle(moved, (76, 72), 25, (255, 80, 20), -1)
        _, regions = guide.detect(moved, selection.current.box)
        selection.update(regions)
        self.assertEqual(selection.state, "tracking")
        self.assertGreater(selection.current.center[0], 73)
        _, regions = guide.detect(np.zeros_like(self.frame), selection.current.box)
        selection.update(regions)
        self.assertIsNone(selection.current)
        self.assertEqual(selection.state, "lost")

    def test_stale_resized_or_missing_target_cannot_be_confirmed(self):
        target = gs.parse_target(target_json())
        for proposal, frame in (
                (gs.Proposal(self.frame, target, time.monotonic() - 46), self.frame),
                (gs.Proposal(self.frame, target, time.monotonic()), np.zeros((90, 120, 3), np.uint8)),
                (gs.Proposal(self.frame, None, time.monotonic()), self.frame)):
            with self.subTest(proposal=proposal.target), self.assertRaises(gs.GeminiError):
                gs.confirm_target(proposal, frame)

    def test_single_foreground_in_loose_proposal_can_be_confirmed(self):
        frame = np.full((320, 320, 3), 190, np.uint8)
        cv2.rectangle(frame, (100, 60), (159, 219), (210, 70, 20), -1)
        target = gs.Target("bottle", (62, 187, 813, 625), "one bottle")
        guide, selected = gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)
        self.assertEqual(selected.state, "tracking")
        self.assertLess(overlap(selected.current.anchor.box, target.pixel_box(frame.shape)), 0.35)
        self.assertEqual(selected.current.box, target.pixel_box(frame.shape))
        guide.follow(frame, selected)
        self.assertEqual(selected.state, "tracking")
        self.assertAlmostEqual(selected.current.center[0], 129.5, delta=2)

    def test_multiple_foreground_regions_in_proposal_are_rejected(self):
        frame = np.zeros((180, 240, 3), np.uint8)
        cv2.rectangle(frame, (50, 40), (100, 110), (255, 80, 20), -1)
        cv2.rectangle(frame, (115, 55), (127, 85), (255, 80, 20), -1)
        target = gs.Target("bottle", (150, 150, 700, 600), "ambiguous foreground")
        with self.assertRaisesRegex(gs.GeminiError, "Multiple foreground"):
            gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)

    def test_empty_snapshot_box_cannot_seed_a_target(self):
        frame = np.zeros_like(self.frame)
        proposal = gs.Proposal(frame, gs.parse_target(target_json()), time.monotonic())
        with self.assertRaisesRegex(gs.GeminiError, "No foreground"):
            gs.confirm_target(proposal, frame)

    def test_fragmented_bottle_uses_dominant_anchor_and_retains_confirmed_box(self):
        frame = fragmented_bottle_scene()
        target = gs.Target("bottle", (150, 200, 850, 480), "one bottle")
        guide, selected = gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)
        self.assertEqual(selected.state, "tracking")
        self.assertEqual(selected.current.box, target.pixel_box(frame.shape))
        self.assertEqual(selected.current.anchor.box, (80, 100, 50, 50))
        self.assertIn("anchor", selected.reason)
        self.assertEqual(guide.mask()[95, 105], 0)  # No invented foreground bridge.
        for shift in (15, 40, 70):
            guide.follow(fragmented_bottle_scene(shift), selected, min_pixels=80)
            self.assertEqual(selected.state, "tracking")
            expected = target.pixel_box(frame.shape)
            self.assertEqual(selected.current.box, (expected[0] + shift, *expected[1:]))
        self.assertEqual(selected.number, 1)

    def test_reflections_cannot_replace_missing_anchor_or_automatically_reacquire(self):
        frame = fragmented_bottle_scene()
        target = gs.Target("bottle", (150, 200, 850, 480), "one bottle")
        guide, selected = gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)
        guide.follow(fragmented_bottle_scene(label=False), selected, min_pixels=80)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)
        _, regions = guide.follow(frame, selected, min_pixels=80)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)
        self.assertEqual(len(regions), 1)
        selected.choose((105, 125), regions)  # Deliberate reselection is still possible.
        guide.follow(frame, selected, min_pixels=80)
        self.assertEqual(selected.state, "tracking")

    def test_fragmented_anchor_removed_before_confirmation_is_rejected(self):
        frame = fragmented_bottle_scene()
        target = gs.Target("bottle", (150, 200, 850, 480), "one bottle")
        with self.assertRaisesRegex(gs.GeminiError, "Target moved or is ambiguous"):
            gs.confirm_target(gs.Proposal(frame, target, time.monotonic()),
                              fragmented_bottle_scene(label=False))

    def test_comparable_foreground_parts_are_not_silently_combined(self):
        frame = fragmented_bottle_scene()
        frame[45:70, 80:130] = (200, 40, 30)
        target = gs.Target("two parts", (150, 200, 850, 480), "ambiguous")
        with self.assertRaisesRegex(gs.GeminiError, "Multiple foreground"):
            gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)

    def test_two_plausible_new_anchors_are_ambiguous_during_tracking(self):
        frame = fragmented_bottle_scene()
        target = gs.Target("bottle", (150, 200, 850, 480), "one bottle")
        guide, selected = gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)
        changed = np.full_like(frame, 180)
        changed[100:150, 20:70] = (200, 40, 30)
        changed[100:150, 160:210] = (200, 40, 30)
        guide.follow(changed, selected, min_pixels=80)
        self.assertEqual(selected.state, "ambiguous")
        self.assertIsNone(selected.current)

    def test_box_leaving_frame_is_lost_even_if_anchor_remains_visible(self):
        frame = fragmented_bottle_scene()
        target = gs.Target("bottle", (150, 200, 850, 480), "one bottle")
        guide, selected = gs.confirm_target(gs.Proposal(frame, target, time.monotonic()), frame)
        for shift in (50, 100, 150):
            guide.follow(fragmented_bottle_scene(shift), selected, min_pixels=80)
            self.assertEqual(selected.state, "tracking")
        guide.follow(fragmented_bottle_scene(170), selected, min_pixels=80)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)

    def test_confirmed_gemini_target_follows_larger_motion_without_another_request(self):
        proposal = gs.Proposal(self.frame.copy(), gs.parse_target(target_json()), time.monotonic())
        guide, selected = gs.confirm_target(proposal, self.frame)
        moved = np.zeros_like(self.frame)
        cv2.circle(moved, (120, 72), 25, (255, 80, 20), -1)
        cv2.circle(moved, (175, 75), 25, (0, 180, 255), -1)
        with patch.object(gs, "select_target") as request:
            guide.follow(moved, selected)
        self.assertEqual(selected.state, "tracking")
        self.assertAlmostEqual(selected.current.center[0], 120, delta=1)
        request.assert_not_called()

    def test_moving_target_merged_with_neighbor_is_not_accepted(self):
        proposal = gs.Proposal(self.frame.copy(), gs.parse_target(target_json()), time.monotonic())
        guide, selected = gs.confirm_target(proposal, self.frame)
        moved = np.zeros_like(self.frame)
        cv2.circle(moved, (125, 72), 25, (255, 80, 20), -1)
        cv2.circle(moved, (175, 75), 25, (0, 180, 255), -1)
        guide.follow(moved, selected)
        self.assertEqual(selected.state, "lost")
        self.assertIsNone(selected.current)

    def test_object_removed_during_cloud_request_cannot_be_confirmed(self):
        proposal = gs.Proposal(self.frame, gs.parse_target(target_json()), time.monotonic())
        with self.assertRaises(gs.GeminiError):
            gs.confirm_target(proposal, np.zeros_like(self.frame))


if __name__ == "__main__":
    unittest.main()
