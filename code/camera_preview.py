"""Webcam contour boxes, operator selection, and uncalibrated image alignment."""

import argparse
import sys
import time

import cv2
import numpy as np

from contour_tracking import Selection, find_regions
from guided_foreground import choose_foreground
from gemini_selection import GeminiError, GeminiSelector, confirm_target
from sensor_telemetry import SensorReader, format_snapshot, serial_port_lines
from motor_control import motor_lines
from manual_motors import ManualControl
from operator_guidance import Correspondence, guidance, load_calibration
from preview_layout import DEFAULT_SIZE, PreviewLayout, text_lines
from auto_capture import AutoCapture, CaptureContext, load_auto_capture_config


WINDOW = "Debris Capture - Camera Preview"
PROPOSAL_WINDOW = "Gemini proposal - saved image, NOT live"


def control_buttons(shape, size=DEFAULT_SIZE):
    return PreviewLayout(shape, size).buttons


def control_at(x, y, shape, size=DEFAULT_SIZE):
    return PreviewLayout(shape, size).control_at(x, y)


def preview_point(x, y, shape, size=DEFAULT_SIZE):
    return PreviewLayout(shape, size).camera_point(x, y)


def display_size(window=WINDOW):
    """Match the displayed bitmap to WIN32's image client, avoiding a second stretch."""
    try:
        _, _, width, height = cv2.getWindowImageRect(window)
        return (width, height) if width > 0 and height > 0 else DEFAULT_SIZE
    except cv2.error:
        return DEFAULT_SIZE


def camera_number(value):
    """Reject invalid device indices before opening any hardware."""
    try:
        index = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Camera number must be 0 or a positive integer.") from exc
    if index < 0:
        raise argparse.ArgumentTypeError("Camera number must be 0 or a positive integer.")
    return index


def draw_view(frame, mask, regions, selection, roi, count, show_mask, dark, sensor_snapshot=None, guided=False,
              ai_status=None, motor_snapshot=None, calibration=None, correspondence=False,
              control_notice="", confirm_reference=False, display_size=DEFAULT_SIZE,
              auto_status="Auto off", auto_enabled=False, manual_enabled=False, stroke_entries=None, stroke_edit=None):
    view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else frame.copy()
    height, width = frame.shape[:2]
    if roi is not None:
        x, y, w, h = roi
        cv2.rectangle(view, (x, y), (x + w - 1, y + h - 1), (180, 180, 180), 1)
    for region in regions:
        selected = region is selection.current
        color = (0, 255, 0) if selected else (255, 200, 0)
        x, y, w, h = region.box
        cv2.drawContours(view, [region.outline], -1, color, 1)
        cv2.rectangle(view, (x, y), (x + w - 1, y + h - 1), color, 2)
    calibrated = calibration is not None and calibration.matches(frame.shape)
    crosshair = tuple(round(v) for v in calibration.crosshair) if calibrated else (width // 2, height // 2)
    cv2.drawMarker(view, crosshair, (160, 160, 160), cv2.MARKER_CROSS, 20, 1)
    if calibrated:
        tx, ty = calibration.tolerance
        cv2.rectangle(view, (round(crosshair[0] - tx), round(crosshair[1] - ty)),
                      (round(crosshair[0] + tx), round(crosshair[1] + ty)), (180, 180, 60), 1)
    if selection.current is not None:
        center = tuple(round(value) for value in selection.current.center)
        cv2.circle(view, center, 4, (0, 255, 0), -1)
        cv2.line(view, crosshair, center, (0, 255, 0), 1)
    layout = PreviewLayout(frame.shape, display_size)
    canvas = np.full((display_size[1], display_size[0], 3), (243, 246, 244), np.uint8)
    left, top, dw, dh = layout.image
    canvas[top:top + dh, left:left + dw] = cv2.resize(view, (dw, dh), interpolation=cv2.INTER_AREA)
    u = layout.unit
    font = cv2.FONT_HERSHEY_SIMPLEX

    def label(text, x, y, available, scale=.45, color=(49, 67, 56), limit=1):
        for i, line in enumerate(text_lines(text, available, scale * u, limit)):
            cv2.putText(canvas, line, (round(x), round(y + i * 17 * u)), font,
                        scale * u, color, 1, cv2.LINE_AA)

    mode = "guided foreground" if guided else ("dark threshold" if dark else "bright threshold")
    label("Debris Capture", 12 * u, 23 * u, display_size[0] - 24 * u, .62)
    label(f"CAMERA   {width} x {height}   |   {mode}   |   Frame {count}",
          12 * u, 41 * u, display_size[0] - 24 * u, .36, (91, 111, 97))
    px, py, pw, ph = layout.panel
    cv2.rectangle(canvas, (px, py), (px + pw - 1, py + ph - 1), (233, 240, 235), -1)
    dx, dy, dw, dh = layout.dock
    cv2.line(canvas, (dx, dy), (dx + dw, dy), (207, 217, 211), 1)
    label("SETUP", dx, dy + 18 * u, dw * .55, .39, (91, 111, 97))
    label("MOVE", dx + round(dw * .57), dy + 18 * u, dw * .43, .39, (91, 111, 97))
    for command, text, (x, y, bw, bh) in layout.buttons:
        if command == "REFERENCE_NEUTRAL" and (confirm_reference is True or confirm_reference == command):
            text = "Confirm physical neutral"
        if command == "REFERENCE_REST" and confirm_reference == command:
            text = "Confirm retracted"
        if command == "ARM":
            text = "Manual arm: ON" if manual_enabled else "Manual arm: OFF"
        if command in ("STROKE_A", "STROKE_B"):
            axis = command[-1]
            text = f"{axis} pulses: {(stroke_entries or {}).get(axis, '900')}" + ("_" if stroke_edit == axis else "")
        if command == "ECHO" and correspondence:
            text = "Clear echo confirmation"
        if command == "AUTO" and auto_enabled:
            text = "Cancel auto"
        color = (64, 48, 181) if command == "STOP" else ((190, 219, 202) if command == "GRAB" else (255, 255, 255))
        cv2.rectangle(canvas, (x, y), (x + bw - 1, y + bh - 1), color, -1)
        cv2.rectangle(canvas, (x, y), (x + bw - 1, y + bh - 1),
                      color if command in ("STOP", "GRAB") else (201, 214, 206), 1)
        scale = (.48 if command in ("GRAB", "NEUTRAL", "STOP") else .38) * u
        while cv2.getTextSize(text, font, scale, 1)[0][0] > bw - 10 * u and scale > .23 * u:
            scale -= .01 * u
        text_width = cv2.getTextSize(text, font, scale, 1)[0][0]
        cv2.putText(canvas, text, (round(x + (bw - text_width) / 2), round(y + bh / 2 + 5 * u)),
                    font, scale, (255, 255, 255) if command == "STOP" else (49, 67, 56), 1, cv2.LINE_AA)
    label("References and Apply do not move. Signed pulses reset on board restart. Manual arm enables moves; STOP turns it off.", dx, dy + 161 * u, dw, .36, (91, 111, 97))
    label("A: ask Gemini   G: outline   R: region   M: mask   T/B: threshold   C: clear   F: full   Q: quit",
          12 * u, display_size[1] - 12 * u, display_size[0] - 24 * u, .36, (91, 111, 97))

    target_detail = selection.reason
    if selection.current is not None:
        dx = selection.current.center[0] - crosshair[0]
        dy = selection.current.center[1] - crosshair[1]
        target_detail = f"dx {dx:+.0f}, dy {dy:+.0f} px | " + ("calibrated" if calibrated else "image reference only")
        if getattr(selection.current, "anchor", None) is not None:
            target_detail = "Estimated box from local anchor | " + target_detail
    sensor_text = "Ultrasonic off" if sensor_snapshot is None else " | ".join(format_snapshot(sensor_snapshot))
    if correspondence:
        sensor_text = sensor_text.replace("echo target unconfirmed", "echo: confirmed target")
    motor_text = "Controller off" if motor_snapshot is None else " | ".join(motor_lines(motor_snapshot))
    if manual_enabled:
        motor_text = motor_text.replace("next move unarmed", "manual arm ON (per-click arming)").replace("next move armed", "manual arm ON (per-click arming)")
    result = guidance(calibration, frame.shape,
                      selection.current.center if selection.state == "tracking" and selection.current else None,
                      sensor_snapshot.reading if sensor_snapshot else None, time.monotonic(), correspondence)
    groups = ((f"TARGET: {selection.state.upper()}", target_detail),
              ("GUIDANCE", result.alignment + " | " + result.range_advice),
              ("GEMINI / MANUAL", ai_status or "A: ask Gemini | G/click: manual selection"),
              ("ULTRASONIC / RANGE", sensor_text),
              ("CONTROLLER", motor_text),
              ("SETUP / LAST ACTION", auto_status + " | " + (control_notice or "Declare neutral or retracted, switch Manual arm ON, then click moves. Hold is unconfirmed.")))
    for (title, text), (x, y, available, lines) in zip(groups, layout.status_slots):
        label(title, x, y, available, .37, (80, 111, 93))
        label(text, x, y + 18 * u, available, .43, limit=lines)
    return canvas


def draw_proposal(proposal):
    """Show the exact cloud input, never an old box drawn on a new live frame."""
    view = proposal.frame.copy()
    x, y, w, h = proposal.target.pixel_box(view.shape)
    cv2.rectangle(view, (x, y), (x + w - 1, y + h - 1), (0, 210, 255), 3)
    scale = min(1.0, 800 / view.shape[1], 500 / view.shape[0])
    view = cv2.resize(view, (round(view.shape[1] * scale), round(view.shape[0] * scale)))
    view = cv2.copyMakeBorder(view, 72, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
    label = proposal.target.label.encode("ascii", "replace").decode("ascii")
    for text, row in ((f"Gemini proposes: {label[:80]}", 20),
                      ("SAVED IMAGE: verify the whole object; keep camera and scene still", 42),
                      ("Enter: confirm for local tracking | C: reject | A: new request", 64)):
        cv2.putText(view, text, (8, row), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
                    (235, 235, 235), 1, cv2.LINE_AA)
    return view


def run_preview(index, port="auto", width=1280, height=720, backend="auto", target=None, calibration=None,
                auto_config=None):
    if backend == "dshow":
        capture = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    elif backend == "v4l2":
        capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
    else:
        capture = cv2.VideoCapture(index)
    sensors = SensorReader(port)
    gemini = GeminiSelector()
    automatic = AutoCapture(auto_config)
    manual = ManualControl(sensors)
    try:
        if not capture.isOpened():
            raise RuntimeError(
                f"Cannot open camera {index}. Check its USB connection, close other "
                "camera apps, and try another camera number."
            )

        # Requests only: the actual camera may negotiate a different resolution.
        if backend in ("dshow", "v4l2"):
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        print(f"Previewing camera {index} (backend {backend}). Verify the selected camera's image.")
        print("Use a contrasting matte background and separated objects.")
        print("R: bench region; sliders: mask settings; click: select; M: mask; B: invert.")
        print("G: outline whole target, then paint keep/remove corrections; T: threshold mode.")
        print("A: type a Gemini target request, then Enter sends one camera snapshot to Google.")
        print("Verify the saved-image proposal; Enter confirms it for local tracking. C rejects.")
        print("Right-click/C: clear; F: full frame; Q/Escape: quit.")
        print(f"Controller/sensors: port {port}, 115200 baud. --port off is camera-only.")
        print("Declare physical neutral OR already-retracted Rest (confirm, no motion). Toggle Manual arm ON, then click Rest/Grab. STOP turns it off.")
        print("Each deliberate move gets its own acknowledged ARM internally. Completion is hold-unconfirmed.")
        print("Auto is off at startup; Enable auto requires measured settings and commissioned guarded firmware.")
        sensors.start()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, *DEFAULT_SIZE)
        cv2.createTrackbar("Threshold", WINDOW, 180, 255, lambda value: None)
        cv2.createTrackbar("Min pixels", WINDOW, 80, 5000, lambda value: None)
        selection = Selection()
        regions = []
        roi = None
        dark = False
        show_mask = False
        last_settings = None
        guide = None
        guide_box = None
        frame_shape = None
        current_display_size = DEFAULT_SIZE
        proposal = None
        prompt_text = None
        correspondence = Correspondence()
        frame_received_at = 0.0
        manual_enabled = False
        manual_session = None
        control_notice = "Guidance calibration unset" if calibration is None else "Measured calibration loaded; verify the physical mount"
        ai_status = "Gemini: A to type a target request | manual fallback: G or click"

        def cancel_auto(reason):
            if automatic.cancel(reason):
                sensors.command("STOP")

        def capture_context(echo_confirmed=None):
            now = time.monotonic()
            if echo_confirmed is None:
                echo_confirmed = correspondence.valid(calibration, frame_shape, selection,
                                                       sensors.snapshot.reading, now)
            return CaptureContext(
                calibration=calibration, frame_shape=frame_shape,
                frame_received_at=frame_received_at, target_number=selection.number,
                target_center=selection.current.center if selection.current is not None else None,
                tracking=selection.state == "tracking", reading=sensors.snapshot.reading,
                correspondence=echo_confirmed, motor=sensors.motor_snapshot,
            )

        def automatic_write_allowed(command, token):
            now = time.monotonic()
            reading = sensors.snapshot.reading
            if not correspondence.matches(calibration, frame_shape, selection, reading, now):
                automatic.cancel("Echo correspondence changed before write")
                return False
            return automatic.write_allowed(command, now, token,
                                           motor=sensors.motor_snapshot, reading=reading)

        def discard_proposal():
            nonlocal proposal
            if proposal is not None:
                try:
                    cv2.destroyWindow(PROPOSAL_WINDOW)
                except cv2.error:
                    pass  # The operator may already have closed this window.
            proposal = None
            gemini.cancel()

        def request_target(frame, text):
            nonlocal ai_status
            cancel_auto("New target request")
            discard_proposal()
            selection.clear("New Gemini request; awaiting operator confirmation")
            try:
                gemini.submit(frame, text)
                ai_status = "Gemini: waiting for target proposal; keep scene still | C: cancel"
            except GeminiError as exc:
                ai_status = str(exc)
                print(f"Gemini: {exc}", file=sys.stderr)

        def on_mouse(event, x, y, flags, param):
            nonlocal ai_status, manual_enabled, manual_session, control_notice
            if event == cv2.EVENT_LBUTTONDOWN:
                action = control_at(x, y, frame_shape, current_display_size) if frame_shape is not None else None
                if action is not None:
                    # Manual Grab/Rest/STOP remain independent of camera selection and auto gates.
                    if action == "AUTO":
                        if manual_enabled or manual.after_arm is not None:
                            manual.stop()
                        manual_enabled = False
                        if automatic.state in ("GUARDING", "WAITING", "ARMING"):
                            cancel_auto("Operator canceled auto")
                        elif prompt_text is not None or proposal is not None:
                            control_notice = "Finish target confirmation before enabling auto"
                        else:
                            automatic.enable(capture_context(), time.monotonic())
                            control_notice = automatic.message
                        return
                    cancel_auto("Manual control changed")
                    if action == "ECHO":
                        if correspondence.target is not None:
                            correspondence.clear()
                            control_notice = "Echo correspondence cleared"
                        else:
                            confirmed = correspondence.confirm(calibration, frame_shape, selection,
                                                               sensors.snapshot.reading, time.monotonic())
                            control_notice = ("Operator confirms echo is this target; clear on scene change"
                                              if confirmed else "Echo confirmation unavailable: select target, calibrate, obtain fresh range")
                    elif action in ("REFERENCE_NEUTRAL", "REFERENCE_REST"):
                        manual_enabled = False
                        if manual.after_arm is not None:
                            manual.stop()
                        manual.reference(time.monotonic(), retracted=action == "REFERENCE_REST")
                        control_notice = manual.notice
                    elif action in ("STROKE_A", "STROKE_B"):
                        manual.action(action, time.monotonic())
                        control_notice = manual.notice
                    elif action == "APPLY_STROKES":
                        manual_enabled = False
                        if manual.after_arm is not None:
                            manual.stop()
                        manual.apply_strokes(time.monotonic())
                        control_notice = manual.notice
                    elif action == "ARM":
                        now = time.monotonic()
                        status = sensors.motor_snapshot
                        manual.clear_action()
                        if manual_enabled:
                            manual.stop()
                            manual_enabled = False
                            control_notice = "Manual arm OFF; STOP requested."
                        elif (not status.fresh(now) or status.pending not in (None, "STATUS")
                              or status.status.moving or status.status.armed
                              or not status.status.configured or not status.status.referenced
                              or not status.manual_profile_ready):
                            control_notice = "Manual arm unavailable: need latest firmware and a fresh, idle, declared reference."
                        else:
                            manual_enabled = True
                            manual_session = status.session_id
                            control_notice = "Manual arm ON. Click Grab or Prepare / Return Rest; enabling sends no motion."
                    elif action in ("GRAB", "NEUTRAL"):
                        manual.stroke_edit = None
                        if manual_enabled:
                            manual.move(action, time.monotonic())
                            control_notice = manual.notice
                        else:
                            control_notice = "Switch Manual arm ON before a movement."
                    elif action in ("JOG_A_PLUS", "JOG_A_MINUS", "JOG_B_PLUS", "JOG_B_MINUS"):
                        manual.stroke_edit = None
                        if manual_enabled:
                            manual.jog(action[4], 100 if action.endswith("PLUS") else -100, time.monotonic())
                            control_notice = manual.notice
                        else:
                            control_notice = "Switch Manual arm ON before a live adjustment."
                    elif action == "STOP":
                        manual_enabled = False
                        manual.stop()
                        control_notice = manual.notice
                    else:
                        manual.clear_action()
                        sensors.command(action)
                        control_notice = sensors.motor_snapshot.message
                    return
                point = preview_point(x, y, frame_shape, current_display_size) if frame_shape is not None else None
                if point is not None:
                    cancel_auto("Target selection changed")
                    correspondence.clear()
                    discard_proposal()
                    selection.choose(point, regions)
                    ai_status = "Manual selection | A to ask Gemini"
            elif event == cv2.EVENT_RBUTTONDOWN and frame_shape is not None and preview_point(x, y, frame_shape, current_display_size) is not None:
                cancel_auto("Target selection cleared")
                correspondence.clear()
                discard_proposal()
                selection.clear()
                ai_status = "Selection cleared | A to ask Gemini"

        cv2.setMouseCallback(WINDOW, on_mouse)
        count = 0
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("Camera stopped providing frames. Check the connection and restart.")
            frame_received_at = time.monotonic()
            count += 1
            height, width = frame.shape[:2]
            frame_shape = frame.shape
            if count == 1:
                print(f"Received video: {width} x {height} pixels.")
                if target is not None:
                    request_target(frame, target)
            result = gemini.poll()
            if result is not None:
                if result.error:
                    ai_status = result.error
                    print(f"Gemini: {result.error}", file=sys.stderr)
                elif result.expired:
                    ai_status = "Gemini: snapshot expired; A to request again"
                elif result.target is None:
                    ai_status = "Gemini: no unambiguous match; A to rephrase or G for manual fallback"
                else:
                    proposal = result
                    cv2.namedWindow(PROPOSAL_WINDOW, cv2.WINDOW_AUTOSIZE)
                    cv2.imshow(PROPOSAL_WINDOW, draw_proposal(proposal))
                    ai_status = "Gemini: inspect saved-image proposal | Enter: confirm | C: reject"
            if proposal is not None and (proposal.expired or
                    cv2.getWindowProperty(PROPOSAL_WINDOW, cv2.WND_PROP_VISIBLE) < 1):
                discard_proposal()
                ai_status = "Gemini: proposal closed/expired; A for a fresh request"
            threshold = cv2.getTrackbarPos("Threshold", WINDOW)
            min_pixels = max(1, cv2.getTrackbarPos("Min pixels", WINDOW))
            settings = (threshold if guide is None else None, min_pixels,
                        dark if guide is None else None, roi, id(guide))
            if last_settings is not None and settings != last_settings:
                selection.clear("Mask settings changed; select again")
            last_settings = settings
            if guide is None:
                mask, regions = find_regions(frame, threshold, min_pixels, dark, roi)
                selection.update(regions)
            else:
                if selection.current is not None:
                    guide_box = selection.current.box
                mask, regions = guide.follow(frame, selection, guide_box, min_pixels)
            echo_confirmed = correspondence.valid(calibration, frame.shape, selection,
                                                  sensors.snapshot.reading, time.monotonic())
            command = automatic.tick(capture_context(echo_confirmed), time.monotonic())
            if command is not None:
                if command == "STOP":
                    accepted = sensors.command(command)
                else:
                    token = automatic.token
                    accepted = sensors.command(
                        command, condition=lambda cmd=command, generation=token:
                        automatic_write_allowed(cmd, generation))
                automatic.command_result(command, bool(accepted))
            now = time.monotonic()
            status = sensors.motor_snapshot
            if manual_enabled and (not status.fresh(now) or status.session_id != manual_session
                                   or not status.status.referenced or not status.status.configured):
                manual_enabled = False
                manual.clear_action()
                control_notice = "Manual arm OFF: controller connection/reference changed."
            had_manual_job = manual.after_arm is not None
            manual.tick(now)
            if had_manual_job and manual.after_arm is None:
                control_notice = manual.notice
            current_display_size = display_size()
            cv2.imshow(WINDOW, draw_view(
                frame, mask, regions, selection, roi, count, show_mask, dark, sensors.snapshot,
                guided=guide is not None,
                ai_status=("Ask: " + prompt_text[-50:] + "_ | Enter: send | Esc: cancel"
                           if prompt_text is not None else ai_status),
                motor_snapshot=sensors.motor_snapshot, calibration=calibration,
                correspondence=echo_confirmed, control_notice=control_notice,
                confirm_reference=(manual.reference_action if time.monotonic() <= manual.reference_until else False),
                display_size=current_display_size, manual_enabled=manual_enabled,
                stroke_entries=manual.stroke_entries, stroke_edit=manual.stroke_edit,
                auto_status=f"Auto {automatic.state}: {automatic.message}",
                auto_enabled=automatic.state in ("GUARDING", "WAITING", "ARMING"),
            ))
            key = cv2.waitKey(1) & 0xFF
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return 0
            if manual.stroke_edit is not None:
                manual.stroke_key(key, time.monotonic())
                control_notice = manual.notice
                continue
            if prompt_text is not None:
                if key == 27:
                    prompt_text = None
                elif key in (10, 13):
                    request_target(frame, prompt_text)
                    prompt_text = None
                elif key in (8, 127):
                    prompt_text = prompt_text[:-1]
                elif 32 <= key <= 126 and len(prompt_text) < 200:
                    prompt_text += chr(key)
                continue
            if key in (ord("q"), ord("Q"), 27):
                return 0
            if key in (ord("a"), ord("A")):
                cancel_auto("Target request opened")
                discard_proposal()
                prompt_text = ""
            elif key in (10, 13) and proposal is not None:
                try:
                    picked_guide, picked_selection = confirm_target(proposal, frame, min_pixels)
                    guide, selection = picked_guide, picked_selection
                    guide_box, roi, last_settings = selection.current.box, None, None
                    label = proposal.target.label.encode("ascii", "replace").decode("ascii")
                    ai_status = f"Gemini: {label[:48]} | confirmed | local anchor, estimated box"
                except (ValueError, cv2.error) as exc:
                    ai_status = str(exc) if isinstance(exc, GeminiError) else "Could not learn target; use G"
                    selection.clear("Gemini handoff failed: " + ai_status)
                    print(f"Gemini handoff: {ai_status}", file=sys.stderr)
                discard_proposal()
            elif key in (ord("c"), ord("C")):
                cancel_auto("Target cleared")
                discard_proposal()
                selection.clear()
                ai_status = "Gemini: cleared | A for a new request | G/click: manual fallback"
            elif key in (ord("g"), ord("G")):
                cancel_auto("Foreground setup opened")
                if sensors.motor_snapshot.blocks_dialog(time.monotonic()):
                    control_notice = "G dialog blocked: wait for fresh idle/disarmed status; STOP remains available"
                    continue
                correspondence.clear()
                discard_proposal()
                ai_status = "Manual foreground | A to ask Gemini"
                try:
                    picked_guide = choose_foreground(frame)
                    if picked_guide is not None:
                        guide = picked_guide
                        guide_box = guide.box
                        roi = None
                        selection.clear("Foreground learned; click one target box")
                        cv2.setTrackbarPos("Min pixels", WINDOW, 80)
                except (ValueError, cv2.error) as exc:
                    print(f"Foreground selection: {exc}", file=sys.stderr)
                    selection.clear("Could not learn foreground; press G to redraw")
            elif key in (ord("t"), ord("T")):
                cancel_auto("Tracking mode changed")
                discard_proposal()
                ai_status = "Manual threshold mode | A to ask Gemini"
                guide = None
                selection.clear()
            elif key in (ord("m"), ord("M")):
                show_mask = not show_mask
            elif key in (ord("b"), ord("B")):
                cancel_auto("Mask settings changed")
                discard_proposal()
                ai_status = "Manual mask settings | A to ask Gemini"
                dark = not dark
            elif key in (ord("f"), ord("F")):
                cancel_auto("Tracking region changed")
                discard_proposal()
                ai_status = "Full-frame region | A to ask Gemini"
                roi = None
            elif key in (ord("r"), ord("R")):
                cancel_auto("Region setup opened")
                if sensors.motor_snapshot.blocks_dialog(time.monotonic()):
                    control_notice = "ROI dialog blocked: wait for fresh idle/disarmed status; STOP remains available"
                    continue
                correspondence.clear()
                discard_proposal()
                ai_status = "Manual bench region | A to ask Gemini"
                region_window = "Bench region - Enter confirms, C cancels"
                scale = min(1.0, 800 / width, 600 / height)
                region_image = cv2.resize(frame, (round(width * scale), round(height * scale)))
                try:
                    cv2.namedWindow(region_window, cv2.WINDOW_AUTOSIZE)
                    picked = tuple(map(int, cv2.selectROI(region_window, region_image,
                                                        showCrosshair=True, fromCenter=False)))
                    sx, sy = width / region_image.shape[1], height / region_image.shape[0]
                    picked = tuple(round(value * (sx if i % 2 == 0 else sy)) for i, value in enumerate(picked))
                finally:
                    cv2.destroyWindow(region_window)
                if picked[2] > 0 and picked[3] > 0:
                    roi = picked
                    guide = None
                # Keep the main control handler installed after the modal selector.
                cv2.setMouseCallback(WINDOW, on_mouse)
    except (RuntimeError, ValueError, cv2.error) as exc:
        print(f"Camera error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if automatic.cancel("Preview closed"):
            sensors.command("STOP")
        gemini.cancel()
        sensors.stop()
        capture.release()
        cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=camera_number, help="Device number: 0, 1, 2, ...")
    parser.add_argument("--port", default="auto",
                        help="Controller USB port, auto (UNO R4 WiFi), or off for camera-only")
    parser.add_argument("--guidance-calibration", metavar="JSON", help="Measured camera/gripper calibration; unset by default")
    parser.add_argument("--auto-capture-config", metavar="JSON",
                        help="Validated dwell settings; auto stays off until Enable auto is clicked")
    parser.add_argument("--list-ports", action="store_true",
                        help="List serial ports/USB IDs and exit without opening camera or serial")
    parser.add_argument("--backend", choices=("auto", "dshow", "v4l2"), default="auto",
                        help="Capture API: auto, Windows dshow, or Linux v4l2; explicit backends request MJPEG")
    parser.add_argument("--width", type=camera_number, default=1280, help="Requested capture width (default 1280)")
    parser.add_argument("--height", type=camera_number, default=720, help="Requested capture height (default 720)")
    parser.add_argument("--target", help="Send one Gemini snapshot request after opening, e.g. 'blue bottle'")
    args = parser.parse_args(argv)
    if args.list_ports:
        print("\n".join(serial_port_lines()))
        return 0
    index = args.camera
    if index is None:
        print("Choose a camera number. Start with 0 for the laptop webcam; verify the image.")
        try:
            index = camera_number(input("Camera number [0]: ").strip() or "0")
        except argparse.ArgumentTypeError as exc:
            parser.error(str(exc))
        except (EOFError, KeyboardInterrupt):
            return 0
    if args.width == 0 or args.height == 0:
        parser.error("Capture width and height must be positive.")
    options = {"target": args.target} if args.target is not None else {}
    if args.guidance_calibration:
        try:
            options["calibration"] = load_calibration(args.guidance_calibration)
        except ValueError as exc:
            parser.error(str(exc))
    if args.auto_capture_config:
        try:
            options["auto_config"] = load_auto_capture_config(args.auto_capture_config)
        except ValueError as exc:
            parser.error(str(exc))
    return run_preview(index, args.port, args.width, args.height, args.backend, **options)


if __name__ == "__main__":
    raise SystemExit(main())
