"""Operator-guided color foreground extraction using the installed OpenCV."""

from math import ceil

import cv2
import numpy as np

from contour_tracking import movement_limit, overlap, regions_from_mask


def expand_box(box, shape, margin=None):
    x, y, w, h = box
    margin = max(16, round(max(w, h) * 0.35)) if margin is None else margin
    left, top = max(0, x - margin), max(0, y - margin)
    right, bottom = min(shape[1], x + w + margin), min(shape[0], y + h + margin)
    return left, top, right - left, bottom - top


def small_image(image, limit=320):
    scale = min(1.0, limit / max(image.shape[:2]))
    return cv2.resize(image, (max(1, round(image.shape[1] * scale)),
                              max(1, round(image.shape[0] * scale))))


def touches_edge(box, bounds):
    x, y, w, h = box
    left, top, width, height = bounds
    return x <= left + 1 or y <= top + 1 or x + w >= left + width - 1 or y + h >= top + height - 1


class ForegroundGuide:
    """Learn colors once; keep models fixed while testing nearby foreground."""

    def __init__(self, frame, box):
        x, y, w, h = map(int, box)
        if x < 0 or y < 0 or w < 3 or h < 3 or x + w > frame.shape[1] or y + h > frame.shape[0]:
            raise ValueError("Draw a rectangle around the whole target inside the image.")
        self.shape = frame.shape[:2]
        self.previous_frame = frame.copy()
        self.box = (x, y, w, h)
        self.bounds = expand_box(self.box, frame.shape)
        left, top, width, height = self.bounds
        self.image = small_image(frame[top:top + height, left:left + width])
        self.labels = np.zeros(self.image.shape[:2], np.uint8)
        self.background = np.zeros((1, 65), np.float64)
        self.foreground = np.zeros((1, 65), np.float64)
        sx, sy = self.image.shape[1] / width, self.image.shape[0] / height
        rect = (round((x - left) * sx), round((y - top) * sy),
                max(1, round(w * sx)), max(1, round(h * sy)))
        if rect[0] + rect[2] > self.image.shape[1]:
            rect = (*rect[:2], self.image.shape[1] - rect[0], rect[3])
        if rect[1] + rect[3] > self.image.shape[0]:
            rect = (*rect[:3], self.image.shape[0] - rect[1])
        if rect == (0, 0, self.image.shape[1], self.image.shape[0]):
            raise ValueError("Leave some background outside the target rectangle.")
        cv2.grabCut(self.image, self.labels, rect, self.background, self.foreground, 3,
                    cv2.GC_INIT_WITH_RECT)

    def mask(self):
        left, top, width, height = self.bounds
        local = np.isin(self.labels, (cv2.GC_FGD, cv2.GC_PR_FGD)).astype(np.uint8) * 255
        mask = np.zeros(self.shape, np.uint8)
        mask[top:top + height, left:left + width] = cv2.resize(
            local, (width, height), interpolation=cv2.INTER_NEAREST)
        return mask

    def mark(self, point, keep):
        left, top, width, height = self.bounds
        x, y = point
        if left <= x < left + width and top <= y < top + height:
            mapped = (round((x - left) * self.image.shape[1] / width),
                      round((y - top) * self.image.shape[0] / height))
            cv2.circle(self.labels, mapped, 3, cv2.GC_FGD if keep else cv2.GC_BGD, -1)

    def refine(self):
        if not np.isin(self.labels, (cv2.GC_FGD, cv2.GC_PR_FGD)).any():
            raise ValueError("No foreground remains; mark part of the target with the left button.")
        cv2.grabCut(self.image, self.labels, None, self.background, self.foreground, 2,
                    cv2.GC_INIT_WITH_MASK)

    def detect(self, frame, box=None, min_pixels=20):
        return self._detect_bounds(frame, expand_box(box or self.box, frame.shape), min_pixels)

    def _detect_bounds(self, frame, bounds, min_pixels, limit=320):
        if frame.shape[:2] != self.shape:
            raise ValueError("Camera size changed; draw the target again.")
        x, y, w, h = bounds
        image = small_image(frame[y:y + h, x:x + w], limit)
        labels = np.full(image.shape[:2], cv2.GC_PR_FGD, np.uint8)
        labels[[0, -1], :] = cv2.GC_BGD
        labels[:, [0, -1]] = cv2.GC_BGD
        cv2.grabCut(image, labels, None, self.background.copy(), self.foreground.copy(), 1,
                    cv2.GC_EVAL_FREEZE_MODEL)
        local = np.isin(labels, (cv2.GC_FGD, cv2.GC_PR_FGD)).astype(np.uint8) * 255
        mask = np.zeros(self.shape, np.uint8)
        mask[y:y + h, x:x + w] = cv2.resize(local, (w, h), interpolation=cv2.INTER_NEAREST)
        return mask, regions_from_mask(mask, min_pixels, bounds)

    def follow(self, frame, selection, box=None, min_pixels=20):
        """Try nearby first, then one bounded wider search before declaring loss."""
        if selection.state != "tracking" or selection.current is None:
            result = self.detect(frame, box, min_pixels)
            self.previous_frame = frame.copy()
            return result  # A lost target still requires deliberate reselection.

        previous = selection.current
        bounds = expand_box(previous.box, frame.shape)
        mask, regions = self.detect(frame, previous.box, min_pixels)
        matches = selection.matches(regions)
        if len(matches) > 1 or (len(matches) == 1 and not touches_edge(matches[0].box, bounds)):
            selection.update(regions)
        else:
            margin = ceil(movement_limit(previous.box) + max(previous.box[2:]) / 2)
            bounds = expand_box(previous.box, frame.shape, margin)
            # Keep more detail in the occasional larger crop, especially for narrow objects.
            mask, regions = self._detect_bounds(frame, bounds, min_pixels, limit=640)
            matches = selection.matches(regions, moving=True)
            if matches:
                _, old_regions = self._detect_bounds(self.previous_frame, bounds, min_pixels, limit=640)
                neighbors = [r for r in old_regions if overlap(r.box, previous.box) < 0.1]
                matches = [r for r in matches if not touches_edge(r.box, bounds)
                           and not any(overlap(r.box, neighbor.box) >= 0.5 for neighbor in neighbors)]
            selection.update(matches, moving=True)
        self.previous_frame = frame.copy()
        return mask, regions


def choose_foreground(frame):
    """Freeze for outlining/correction; the serial worker remains independent."""
    window = "Outline whole target - Enter confirms, C cancels"
    scale = min(1.0, 640 / frame.shape[1], 480 / frame.shape[0])
    image = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)))
    sx, sy = frame.shape[1] / image.shape[1], frame.shape[0] / image.shape[0]
    try:
        picked = cv2.selectROI(window, image, showCrosshair=True, fromCenter=False)
        if picked[2] < 3 or picked[3] < 3:
            return None
        guide = ForegroundGuide(frame, tuple(round(v * (sx if i % 2 == 0 else sy))
                                             for i, v in enumerate(picked)))

        def paint(event, x, y, flags, param):
            point = (round(x * sx), round((y - 52) * sy))
            if event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
                guide.mark(point, True)
            elif event == cv2.EVENT_RBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_RBUTTON):
                guide.mark(point, False)

        cv2.setMouseCallback(window, paint)
        message = "Left-drag: keep top/body | Right-drag: remove shadow"
        while True:
            mask = cv2.resize(guide.mask(), (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
            view = image.copy()
            view[mask > 0] = (view[mask > 0].astype(np.float32) * 0.55 + np.array([0, 115, 0])).clip(0, 255).astype(np.uint8)
            canvas = cv2.copyMakeBorder(view, 52, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25, 25, 25))
            for text, row in ((message, 19), ("Space: refine | Enter: use | Esc: cancel", 41)):
                cv2.putText(canvas, text, (8, row), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (240, 240, 240), 1, cv2.LINE_AA)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(20) & 0xFF
            if key in (13, 32):
                try:
                    guide.refine()
                    message = "Green overlay is foreground; correct missing parts or shadow"
                except (ValueError, cv2.error):
                    message = "No usable foreground; left-paint the target or Esc to redraw"
                    continue
                if key == 13:
                    return guide
            if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                return None
    finally:
        cv2.destroyWindow(window)
