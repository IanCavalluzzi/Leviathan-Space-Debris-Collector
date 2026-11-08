"""Exercise camera failure and cleanup without opening a physical camera."""

import argparse
import time
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from unittest.mock import MagicMock, patch

import numpy as np

import camera_preview
from gemini_selection import Proposal, Target
from sensor_telemetry import SensorSnapshot, parse_line
from motor_control import MotorSnapshot, parse_motor_line
from preview_layout import DEFAULT_SIZE, PreviewLayout, text_lines


class CameraPreviewTests(unittest.TestCase):
    def setUp(self):
        sensor_reader = patch.object(camera_preview, "SensorReader")
        mocked_reader = sensor_reader.start()
        mocked_reader.return_value.snapshot = SensorSnapshot("off")
        mocked_reader.return_value.motor_snapshot = MotorSnapshot()
        self.reader = mocked_reader.return_value
        self.addCleanup(sensor_reader.stop)
        geometry = patch.object(camera_preview.cv2, "getWindowImageRect", return_value=(0, 0, *DEFAULT_SIZE))
        geometry.start()
        self.addCleanup(geometry.stop)
        for name in ("namedWindow", "resizeWindow", "createTrackbar", "setTrackbarPos", "setMouseCallback"):
            patcher = patch.object(camera_preview.cv2, name)
            patcher.start()
            self.addCleanup(patcher.stop)
        trackbars = patch.object(
            camera_preview.cv2, "getTrackbarPos",
            side_effect=lambda name, window: 180 if name == "Threshold" else 80,
        )
        trackbars.start()
        self.addCleanup(trackbars.stop)

    def test_rejects_invalid_camera_numbers(self):
        for value in ("-1", "usb", "0.5"):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                camera_preview.camera_number(value)

    def test_unavailable_camera_is_reported_and_released(self):
        capture = MagicMock()
        capture.isOpened.return_value = False
        errors = StringIO()
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stderr(errors):
            self.assertEqual(camera_preview.run_preview(2), 1)
        self.assertIn("Cannot open camera 2", errors.getvalue())
        capture.release.assert_called_once()

    def test_disconnected_camera_is_reported_and_released(self):
        capture = MagicMock()
        capture.read.return_value = (False, None)
        errors = StringIO()
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), \
                redirect_stdout(StringIO()), redirect_stderr(errors):
            self.assertEqual(camera_preview.run_preview(1), 1)
        self.assertIn("stopped providing frames", errors.getvalue())
        capture.release.assert_called_once()

    def test_quit_key_releases_camera_and_windows(self):
        capture = MagicMock()
        capture.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", return_value=ord("q")), \
                patch.object(camera_preview.cv2, "destroyAllWindows") as close_windows, \
                redirect_stdout(StringIO()):
            self.assertEqual(camera_preview.run_preview(0), 0)
        capture.release.assert_called_once()
        close_windows.assert_called_once()

    def test_sensor_overlay_uses_reading_without_mutating_camera_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        mask = np.zeros((480, 640), dtype=np.uint8)
        snapshot = SensorSnapshot("receiving", "COM22", parse_line("1000,580,OK,10.0,0,1,0,1,0", 100.0))
        with patch("sensor_telemetry.time.monotonic", return_value=100.1), \
                patch.object(camera_preview.cv2, "putText", wraps=camera_preview.cv2.putText) as text:
            rendered = camera_preview.draw_view(
                frame, mask, [], camera_preview.Selection(), None, 1, False, False, snapshot
            )
        self.assertEqual(rendered.shape, (800, 1100, 3))
        self.assertFalse(frame.any())
        self.assertTrue(any("10.0 cm" in call.args[1] for call in text.call_args_list))
        self.assertFalse(any("IR raw" in call.args[1] or "OUT1" in call.args[1]
                             for call in text.call_args_list))

    def test_status_panels_do_not_cover_top_or_bottom_of_hd_image(self):
        frame = np.full((720, 1280, 3), (31, 72, 140), dtype=np.uint8)
        rendered = camera_preview.draw_view(
            frame, np.zeros((720, 1280), np.uint8), [], camera_preview.Selection(),
            None, 1, False, False,
        )
        self.assertEqual(rendered.shape, (800, 1100, 3))
        x, y, w, h = PreviewLayout(frame.shape).image
        np.testing.assert_array_equal(rendered[y, x], frame[0, 0])
        np.testing.assert_array_equal(rendered[y + h - 1, x + w - 1], frame[-1, -1])

    def test_selection_coordinates_exclude_status_panels_at_both_sizes(self):
        for height, width in ((480, 640), (720, 1280)):
            layout = PreviewLayout((height, width, 3))
            x, y = layout.display_point(15, 21)
            point = camera_preview.preview_point(x, y, layout.shape)
            self.assertLessEqual(abs(point[0] - 15), 1)
            self.assertLessEqual(abs(point[1] - 21), 1)
            x, y, w, h = layout.image
            self.assertIsNone(camera_preview.preview_point(x, y - 1, layout.shape))
            self.assertIsNone(camera_preview.preview_point(x, y + h, layout.shape))

    def test_hd_capture_is_requested_and_resources_are_released(self):
        capture = MagicMock()
        capture.read.return_value = (True, np.zeros((720, 1280, 3), np.uint8))
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", return_value=ord("q")), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            self.assertEqual(camera_preview.run_preview(0), 0)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.release.assert_called_once()

    def test_cancelled_redraw_keeps_the_active_foreground_model(self):
        frame = np.zeros((720, 1280, 3), np.uint8)
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        guide = MagicMock()
        guide.box = (100, 100, 200, 300)
        guide.follow.return_value = (np.zeros((720, 1280), np.uint8), [])
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "choose_foreground", side_effect=[guide, None]), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[ord("g"), ord("g"), ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            self.assertEqual(camera_preview.run_preview(0), 0)
        self.assertEqual(guide.follow.call_count, 2)
        capture.release.assert_called_once()

    def test_windows_usb_camera_uses_directshow_and_mjpeg(self):
        capture = MagicMock()
        capture.read.return_value = (True, np.zeros((720, 1280, 3), np.uint8))
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture) as open_camera, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", return_value=ord("q")), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            self.assertEqual(camera_preview.run_preview(1, backend="dshow"), 0)
        open_camera.assert_called_once_with(1, camera_preview.cv2.CAP_DSHOW)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FOURCC,
                                    camera_preview.cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_WIDTH, 1280)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_HEIGHT, 720)
        capture.release.assert_called_once()

    def test_usb_backend_option_reaches_preview(self):
        with patch.object(camera_preview, "run_preview", return_value=0) as run:
            self.assertEqual(camera_preview.main(["--camera", "1", "--backend", "dshow"]), 0)
        run.assert_called_once_with(1, "auto", 1280, 720, "dshow")

    def test_linux_usb_camera_uses_v4l2_and_mjpeg(self):
        capture = MagicMock()
        capture.read.return_value = (True, np.zeros((600, 800, 3), np.uint8))
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture) as open_camera, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", return_value=ord("q")), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            self.assertEqual(camera_preview.run_preview(0, port="off", width=800,
                                                        height=600, backend="v4l2"), 0)
        open_camera.assert_called_once_with(0, camera_preview.cv2.CAP_V4L2)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FOURCC,
                                    camera_preview.cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_WIDTH, 800)
        capture.set.assert_any_call(camera_preview.cv2.CAP_PROP_FRAME_HEIGHT, 600)
        camera_preview.SensorReader.assert_called_once_with("off")
        capture.release.assert_called_once()

    def test_linux_backend_option_reaches_preview(self):
        with patch.object(camera_preview, "run_preview", return_value=0) as run:
            self.assertEqual(camera_preview.main(["--camera", "0", "--backend", "v4l2"]), 0)
        run.assert_called_once_with(0, "auto", 1280, 720, "v4l2")

    def test_port_listing_exits_without_prompt_camera_or_serial(self):
        output = StringIO()
        with patch.object(camera_preview, "serial_port_lines", return_value=["COM7: USB UART"]), \
                patch.object(camera_preview, "run_preview") as run, \
                patch.object(camera_preview.cv2, "VideoCapture") as capture, \
                patch("builtins.input") as prompt, redirect_stdout(output):
            self.assertEqual(camera_preview.main(["--list-ports"]), 0)
        self.assertIn("COM7", output.getvalue())
        run.assert_not_called()
        capture.assert_not_called()
        camera_preview.SensorReader.assert_not_called()
        prompt.assert_not_called()

    def test_explicit_usb_port_reaches_single_preview_reader(self):
        with patch.object(camera_preview, "run_preview", return_value=0) as run:
            self.assertEqual(camera_preview.main(["--camera", "1", "--port", "COM7"]), 0)
        run.assert_called_once_with(1, "COM7", 1280, 720, "auto")

    def test_typing_request_keeps_camera_running_and_does_not_interpret_control_letters(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        keys = [ord("a"), *map(ord, "square glass"), 13, ord("q")]
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=keys), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.return_value = None
            self.assertEqual(camera_preview.run_preview(1), 0)
        self.assertEqual(capture.read.call_count, len(keys))
        selector.return_value.submit.assert_called_once_with(frame, "square glass")
        self.reader.command.assert_not_called()

    def run_ui_events(self, events, frame=None, size=DEFAULT_SIZE):
        frame = np.zeros((480, 640, 3), np.uint8) if frame is None else frame
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        events = iter(events)

        def next_key(delay):
            event = next(events)
            if isinstance(event, tuple):
                kind, action = event
                if kind == "click":
                    box = next(box for command, _, box in camera_preview.control_buttons(frame.shape, size) if command == action)
                    x, y, w, h = box
                    callback = camera_preview.cv2.setMouseCallback.call_args.args[1]
                    callback(camera_preview.cv2.EVENT_LBUTTONDOWN, x + w // 2, y + h // 2, 0, None)
                elif kind == "select":
                    callback = camera_preview.cv2.setMouseCallback.call_args.args[1]
                    x, y = PreviewLayout(frame.shape, size).display_point(*action)
                    callback(camera_preview.cv2.EVENT_LBUTTONDOWN, round(x), round(y), 0, None)
                elif kind == "padding":
                    callback = camera_preview.cv2.setMouseCallback.call_args.args[1]
                    callback(action, 0, 0, 0, None)
                elif kind == "loss":
                    frame[:] = 0
                return 255
            return ord(event) if isinstance(event, str) else event

        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview.cv2, "getWindowImageRect", return_value=(0, 0, *size)), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=next_key), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.return_value = None
            self.assertEqual(camera_preview.run_preview(0), 0)

    def test_controls_hit_test_excludes_camera_and_keeps_manual_grab_rest_stop_without_target(self):
        for height, width in ((480, 640), (720, 1280)):
            shape = (height, width, 3)
            for action, _, (x, y, w, h) in camera_preview.control_buttons(shape):
                self.assertEqual(camera_preview.control_at(x + w // 2, y + h // 2, shape), action)
                self.assertIsNone(camera_preview.preview_point(x + w // 2, y + h // 2, shape))
            self.assertIsNone(camera_preview.control_at(50, round(80 * width / 640) + 50, shape))
        self.run_ui_events([("click", "GRAB"), ("click", "STOP"), ("click", "NEUTRAL"), "q"])
        self.assertEqual([call.args[0] for call in self.reader.command.call_args_list], ["STOP"])

    def test_physical_reference_requires_two_clicks_and_idle_disarmed_status(self):
        s = parse_motor_line("# motor UNREFERENCED 1 0 0 0 0 OK 0 0 -1", time.monotonic())
        self.reader.motor_snapshot = MotorSnapshot(True, s, synchronized=True)
        self.run_ui_events([("click", "REFERENCE_NEUTRAL"), ("click", "REFERENCE_NEUTRAL"), "q"])
        self.reader.command.assert_called_once_with("REFERENCE_NEUTRAL")

    def test_physical_reference_clicks_work_during_read_only_status_poll(self):
        s = parse_motor_line("# motor UNREFERENCED 1 0 0 0 0 OK 0 0 -1", time.monotonic())
        self.reader.motor_snapshot = MotorSnapshot(True, s, pending="STATUS", synchronized=True)
        self.run_ui_events([("click", "REFERENCE_NEUTRAL"), ("click", "REFERENCE_NEUTRAL"), "q"])
        self.reader.command.assert_called_once_with("REFERENCE_NEUTRAL")

    def test_retracted_reference_is_below_neutral_and_needs_its_own_confirmation(self):
        from motor_control import PAPER_PROFILE
        s = parse_motor_line("# motor UNREFERENCED 1 0 0 0 0 OK 0 0 -1", time.monotonic())
        self.reader.motor_snapshot = MotorSnapshot(True, s, synchronized=True, controls=True, profile=PAPER_PROFILE)
        self.run_ui_events([("click", "REFERENCE_NEUTRAL"), ("click", "REFERENCE_REST"),
                            ("click", "REFERENCE_REST"), "q"])
        self.reader.command.assert_called_once_with("REFERENCE_REST")
        boxes = {name: box for name, _, box in camera_preview.control_buttons((600, 800, 3))}
        self.assertEqual(boxes["REFERENCE_REST"][0], boxes["REFERENCE_NEUTRAL"][0])
        self.assertGreaterEqual(boxes["REFERENCE_REST"][1], boxes["REFERENCE_NEUTRAL"][1] + boxes["REFERENCE_NEUTRAL"][3])

    def test_auto_button_cannot_move_without_configuration_or_commissioning(self):
        self.run_ui_events([("click", "AUTO"), 255, ("click", "AUTO"), "q"])
        self.reader.command.assert_not_called()

    def test_invalid_auto_config_is_rejected_before_devices_open(self):
        with patch.object(camera_preview, "run_preview") as run, \
                redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            camera_preview.main(["--camera", "0", "--auto-capture-config", "missing-auto-config.json"])
        run.assert_not_called()

    def test_reference_and_blocking_dialogs_are_unavailable_during_motion(self):
        s = parse_motor_line("# motor GRABBING 1 0 1 1 -1 ACCEPTED 1 2 -1", time.monotonic())
        self.reader.motor_snapshot = MotorSnapshot(True, s, synchronized=True)
        with patch.object(camera_preview, "choose_foreground") as choose, \
                patch.object(camera_preview.cv2, "selectROI") as roi:
            self.run_ui_events(["g", "r", ("click", "REFERENCE_NEUTRAL"),
                                ("click", "REFERENCE_NEUTRAL"), ("click", "STOP"), "q"])
        choose.assert_not_called()
        roi.assert_not_called()
        self.reader.command.assert_called_once_with("STOP")

    def test_target_loss_and_wrapping_never_issue_motor_commands(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[100:200, 100:200] = 255
        states = []
        original_draw = camera_preview.draw_view

        def record(*args, **kwargs):
            states.append(args[3].state)
            return original_draw(*args, **kwargs)

        with patch.object(camera_preview, "draw_view", side_effect=record):
            self.run_ui_events([("select", (150, 150)), ("loss", None), 255, "q"], frame)
        self.assertEqual(states, ["unselected", "tracking", "lost", "lost"])
        self.reader.command.assert_not_called()

    def test_motor_buttons_remain_mouse_accessible_while_typing(self):
        self.run_ui_events(["a", "g", "r", "a", "b", ("click", "STOP"), "n", "e", "u", "t", "r", "a", "l", 27, "q"])
        self.reader.command.assert_called_once_with("STOP")

    def test_control_panel_preserves_full_image_and_reports_unset_guidance(self):
        frame = np.full((720, 1280, 3), (31, 72, 140), dtype=np.uint8)
        with patch.object(camera_preview.cv2, "putText", wraps=camera_preview.cv2.putText) as text:
            rendered = camera_preview.draw_view(frame, np.zeros((720, 1280), np.uint8), [],
                                                camera_preview.Selection(), None, 1, False, False,
                                                motor_snapshot=MotorSnapshot())
        x, y, w, h = PreviewLayout(frame.shape).image
        np.testing.assert_array_equal(rendered[y, x], frame[0, 0])
        np.testing.assert_array_equal(rendered[y + h - 1, x + w - 1], frame[-1, -1])
        self.assertEqual(rendered.shape, (800, 1100, 3))
        self.assertTrue(any("unavailable" in call.args[1] for call in text.call_args_list))

    def test_gemini_proposal_needs_confirmation_and_reject_does_not_handoff(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        proposal = Proposal(frame, Target("bottle", (100, 100, 800, 700), "one bottle"), time.monotonic())
        states = []

        def record_view(*args, **kwargs):
            states.append(args[3].state)
            return frame

        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "confirm_target") as confirm, \
                patch.object(camera_preview, "draw_view", side_effect=record_view), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[255, ord("c"), ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.side_effect = [proposal, None, None]
            self.assertEqual(camera_preview.run_preview(1), 0)
        self.assertEqual(states, ["unselected"] * 3)
        confirm.assert_not_called()

    def test_gemini_confirmation_activates_local_guide_without_next_frame_reset(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[100:200, 100:200] = 255
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        mask, regions = camera_preview.find_regions(frame)
        selected = camera_preview.Selection()
        selected.choose((150, 150), regions)
        guide = MagicMock()
        guide.follow.return_value = (mask, regions)
        proposal = Proposal(frame, Target("bottle", (100, 100, 800, 700), "one bottle"), time.monotonic())
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "confirm_target", return_value=(guide, selected)) as confirm, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[13, ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.side_effect = [proposal, None]
            self.assertEqual(camera_preview.run_preview(1), 0)
        confirm.assert_called_once_with(proposal, frame, 80)
        guide.follow.assert_called_once()
        self.assertEqual(selected.state, "tracking")

    def test_real_loose_box_handoff_stays_selected_in_live_loop(self):
        frame = np.full((320, 320, 3), 190, np.uint8)
        frame[60:220, 100:160] = (210, 70, 20)
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        proposal = Proposal(frame, Target("bottle", (62, 187, 813, 625), "one bottle"), time.monotonic())
        states = []

        def record_view(*args, **kwargs):
            states.append((args[3].state, args[3].current))
            return frame

        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "draw_view", side_effect=record_view), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[13, 255, ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.side_effect = [proposal, None, None]
            self.assertEqual(camera_preview.run_preview(1, port="off"), 0)
        self.assertEqual([state for state, region in states], ["unselected", "tracking", "tracking"])
        self.assertTrue(all(region is not None for state, region in states[1:]))
        self.reader.command.assert_not_called()

    def test_fragmented_gemini_handoff_keeps_box_and_marks_it_estimated(self):
        frame = np.full((240, 320, 3), 180, np.uint8)
        frame[100:150, 80:130] = (200, 40, 30)
        frame[70:80, 90:100] = 240
        frame[85:95, 110:120] = 240
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        target = Target("bottle", (150, 200, 850, 480), "one bottle")
        proposal = Proposal(frame, target, time.monotonic())
        states, statuses = [], []

        # Exercise the real renderer as well as real segmentation and handoff.
        renderer = camera_preview.draw_view
        def render(*args, **kwargs):
            selected = args[3]
            states.append((selected.state, selected.current.box if selected.current else None))
            statuses.append(kwargs["ai_status"])
            return renderer(*args, **kwargs)

        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "draw_view", side_effect=render), \
                patch.object(camera_preview.cv2, "putText", wraps=camera_preview.cv2.putText) as texts, \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[13, 255, ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.side_effect = [proposal, None, None]
            self.assertEqual(camera_preview.run_preview(1, port="off"), 0)
        self.assertEqual(states[1:], [("tracking", target.pixel_box(frame.shape))] * 2)
        self.assertIn("estimated box", statuses[-1])
        self.assertTrue(any("Estimated box" in call.args[1] for call in texts.call_args_list))
        self.reader.command.assert_not_called()

    def test_failed_gemini_handoff_keeps_specific_reason_and_logs_it(self):
        frame = np.zeros((180, 240, 3), np.uint8)
        capture = MagicMock()
        capture.read.return_value = (True, frame)
        proposal = Proposal(frame, Target("bottle", (100, 100, 800, 700), "one bottle"), time.monotonic())
        error = "No foreground found in the proposed box; use G to outline the target."
        states = []
        errors = StringIO()

        def record_view(*args, **kwargs):
            states.append((args[3].state, args[3].reason, kwargs["ai_status"]))
            return frame

        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "confirm_target", side_effect=camera_preview.GeminiError(error)), \
                patch.object(camera_preview, "draw_view", side_effect=record_view), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[13, ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), \
                redirect_stdout(StringIO()), redirect_stderr(errors):
            selector.return_value.poll.side_effect = [proposal, None]
            self.assertEqual(camera_preview.run_preview(1, port="off"), 0)
        self.assertEqual(states[-1], ("unselected", "Gemini handoff failed: " + error, error))
        self.assertIn("Gemini handoff: " + error, errors.getvalue())
        self.reader.command.assert_not_called()

    def test_target_cli_request_reaches_preview(self):
        with patch.object(camera_preview, "run_preview", return_value=0) as run:
            self.assertEqual(camera_preview.main(["--camera", "1", "--target", "blue bottle"]), 0)
        run.assert_called_once_with(1, "auto", 1280, 720, "auto", target="blue bottle")

    def test_resized_four_three_ui_keeps_stop_and_reference_mapping(self):
        frame = np.zeros((1536, 2048, 3), np.uint8)
        s = parse_motor_line("# motor UNREFERENCED 1 0 0 0 0 OK 0 0 -1", time.monotonic())
        self.reader.motor_snapshot = MotorSnapshot(True, s, synchronized=True)
        self.run_ui_events([("padding", camera_preview.cv2.EVENT_LBUTTONDOWN),
                            ("padding", camera_preview.cv2.EVENT_RBUTTONDOWN),
                            ("click", "REFERENCE_NEUTRAL"), ("click", "REFERENCE_NEUTRAL"),
                            ("click", "STOP"), "q"], frame, (700, 1000))
        self.assertEqual([c.args[0] for c in self.reader.command.call_args_list], ["REFERENCE_NEUTRAL", "STOP"])

    def test_roi_uses_scaled_autosize_dialog_and_returns_capture_coordinates(self):
        frame = np.zeros((1536, 2048, 3), np.uint8)
        self.reader.motor_snapshot = MotorSnapshot()  # Camera-only dialogs remain permitted.
        capture = MagicMock()
        capture.read.return_value = True, frame
        masks = []
        original = camera_preview.find_regions
        def find(*args):
            masks.append(args[-1])
            return original(*args)
        with patch.object(camera_preview.cv2, "VideoCapture", return_value=capture), \
                patch.object(camera_preview, "GeminiSelector") as selector, \
                patch.object(camera_preview, "find_regions", side_effect=find), \
                patch.object(camera_preview.cv2, "selectROI", return_value=(100, 50, 200, 100)) as pick, \
                patch.object(camera_preview.cv2, "destroyWindow"), \
                patch.object(camera_preview.cv2, "imshow"), \
                patch.object(camera_preview.cv2, "waitKey", side_effect=[ord("r"), ord("q")]), \
                patch.object(camera_preview.cv2, "getWindowProperty", return_value=1), \
                patch.object(camera_preview.cv2, "destroyAllWindows"), redirect_stdout(StringIO()):
            selector.return_value.poll.return_value = None
            self.assertEqual(camera_preview.run_preview(0, port="off"), 0)
        self.assertEqual(pick.call_args.args[1].shape, (600, 800, 3))
        self.assertEqual(masks[-1], (256, 128, 512, 256))
        self.assertEqual(camera_preview.cv2.setMouseCallback.call_count, 2)


class PreviewLayoutTests(unittest.TestCase):
    def test_g_outline_and_correction_keep_four_three_native_mapping(self):
        import guided_foreground
        for width, height in ((2048, 1536), (800, 600)):
            frame = np.zeros((height, width, 3), np.uint8)
            guide = MagicMock()
            guide.mask.return_value = np.zeros((height, width), np.uint8)
            def paint_and_cancel(delay):
                callback = guided_foreground.cv2.setMouseCallback.call_args.args[1]
                callback(cv2.EVENT_LBUTTONDOWN, 160, 172, cv2.EVENT_FLAG_LBUTTON, None)
                return 27
            cv2 = camera_preview.cv2
            with patch.object(guided_foreground.cv2, "selectROI", return_value=(160, 120, 200, 200)) as pick, \
                    patch.object(guided_foreground, "ForegroundGuide", return_value=guide) as create, \
                    patch.object(guided_foreground.cv2, "setMouseCallback"), \
                    patch.object(guided_foreground.cv2, "waitKey", side_effect=paint_and_cancel), \
                    patch.object(guided_foreground.cv2, "imshow"), \
                    patch.object(guided_foreground.cv2, "getWindowProperty", return_value=1), \
                    patch.object(guided_foreground.cv2, "destroyWindow"):
                self.assertIsNone(guided_foreground.choose_foreground(frame))
            self.assertEqual(pick.call_args.args[1].shape, (480, 640, 3))
            self.assertIs(create.call_args.args[0], frame)
            scale = width / 640
            self.assertEqual(create.call_args.args[1], tuple(round(v * scale) for v in (160, 120, 200, 200)))
            guide.mark.assert_called_once_with((round(160 * scale), round(120 * scale)), True)

    def test_aspect_fit_inverse_mapping_and_padding_across_sizes(self):
        for shape in ((1536, 2048, 3), (600, 800, 3), (720, 1280, 3)):
            for size in ((1100, 800), (1440, 600), (700, 1000), (800, 600), (640, 480)):
                with self.subTest(shape=shape, size=size):
                    layout = PreviewLayout(shape, size)
                    x, y, w, h = layout.image
                    self.assertLessEqual(abs(w / shape[1] - h / shape[0]), 1 / min(shape[:2]))
                    self.assertGreaterEqual(x, 0)
                    self.assertGreaterEqual(y, 0)
                    self.assertLessEqual(x + w, size[0])
                    self.assertLessEqual(y + h, size[1])
                    self.assertIsNone(layout.camera_point(x - 1, y))
                    self.assertIsNone(layout.camera_point(x, y - 1))
                    self.assertIsNone(layout.camera_point(x + w, y))
                    self.assertIsNone(layout.camera_point(x, y + h))
                    point = (shape[1] * .31, shape[0] * .67)
                    restored = layout.camera_point(*layout.display_point(*point))
                    self.assertLessEqual(abs(restored[0] - point[0]), 1)
                    self.assertLessEqual(abs(restored[1] - point[1]), 1)
                    for action, _, (bx, by, bw, bh) in layout.buttons:
                        self.assertEqual(layout.control_at(bx + bw // 2, by + bh // 2), action)
                        self.assertIsNone(layout.camera_point(bx + bw // 2, by + bh // 2))
                        self.assertTrue(0 <= bx < bx + bw <= size[0])
                        self.assertTrue(0 <= by < by + bh <= size[1])
                    self.assertIsNone(layout.control_at(0, 0))

    def test_render_keeps_camera_corners_and_original_frame_for_four_three(self):
        for height, width in ((1536, 2048), (600, 800), (720, 1280)):
            frame = np.full((height, width, 3), (31, 72, 140), np.uint8)
            before = frame.copy()
            for size in ((1100, 800), (1440, 600), (700, 1000)):
                rendered = camera_preview.draw_view(frame, np.zeros((height, width), np.uint8), [],
                    camera_preview.Selection(), None, 1, False, False,
                    motor_snapshot=MotorSnapshot(), display_size=size)
                self.assertEqual(rendered.shape, (size[1], size[0], 3))
                x, y, w, h = PreviewLayout(frame.shape, size).image
                for row, col in ((y, x), (y + h - 1, x + w - 1)):
                    np.testing.assert_array_equal(rendered[row, col], frame[0, 0])
            np.testing.assert_array_equal(frame, before)

    def test_status_wrap_is_bounded_by_pixel_width_and_line_count(self):
        for text in ("a long status " * 20, "X" * 200):
            lines = text_lines(text, 180, .45, 2)
            self.assertLessEqual(len(lines), 2)
            self.assertTrue(lines[-1].endswith("..."))
            for line in lines:
                self.assertLessEqual(camera_preview.cv2.getTextSize(line,
                    camera_preview.cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0], 180)

    def test_light_lab_dock_is_below_camera_and_status_and_has_separate_hit_targets(self):
        def overlaps(a, b):
            ax, ay, aw, ah = a
            bx, by, bw, bh = b
            return ax < bx + bw and bx < ax + aw and ay < by + bh and by < ay + ah

        for size in ((1100, 800), (1440, 600), (700, 1000), (800, 600), (640, 480)):
            with self.subTest(size=size):
                layout = PreviewLayout((1536, 2048, 3), size)
                self.assertFalse(overlaps(layout.image, layout.panel))
                self.assertFalse(overlaps(layout.image, layout.dock))
                self.assertFalse(overlaps(layout.panel, layout.dock))
                buttons = layout.buttons
                self.assertEqual({command for command, _, _ in buttons},
                                 {"REFERENCE_NEUTRAL", "REFERENCE_REST", "ECHO", "ARM", "AUTO", "GRAB", "NEUTRAL", "STOP", "STROKE_A", "STROKE_B", "APPLY_STROKES", "JOG_A_PLUS", "JOG_A_MINUS", "JOG_B_PLUS", "JOG_B_MINUS"})
                for i, (command, _, box) in enumerate(buttons):
                    self.assertGreaterEqual(box[1], layout.dock[1])
                    self.assertIsNone(layout.camera_point(box[0], box[1]))
                    for _, _, other in buttons[i + 1:]:
                        self.assertFalse(overlaps(box, other))
                    self.assertEqual(layout.control_at(box[0], box[1]), command)

    def test_reference_confirmation_and_echo_state_fit_the_light_lab_buttons(self):
        frame = np.zeros((600, 800, 3), np.uint8)
        with patch.object(camera_preview.cv2, "putText", wraps=camera_preview.cv2.putText) as text:
            camera_preview.draw_view(frame, np.zeros((600, 800), np.uint8), [],
                                     camera_preview.Selection(), None, 1, False, False,
                                     confirm_reference=True, correspondence=True)
        rendered_text = [call.args[1] for call in text.call_args_list]
        self.assertIn("Confirm physical neutral", rendered_text)
        self.assertIn("Clear echo confirmation", rendered_text)
        self.assertIn("STOP", rendered_text)


if __name__ == "__main__":
    unittest.main()
