"""Display-only geometry. Capture pixels and controller state stay outside this module."""
from dataclasses import dataclass

import cv2

DEFAULT_SIZE = (1100, 800)


@dataclass(frozen=True)
class PreviewLayout:
    shape: tuple
    size: tuple = DEFAULT_SIZE

    @property
    def unit(self):
        return min(1.0, self.size[0] / 640, self.size[1] / 480)

    @property
    def header(self):
        return round(44 * self.unit)

    @property
    def sidebar(self):
        return self.size[0] >= 960

    @property
    def panel(self):
        w, h = self.size
        margin = round(12 * self.unit)
        bottom = self.dock[1] - margin
        if self.sidebar:
            top = self.header + margin
            return w - 252, top, 240, bottom - top
        height = round(min(282 * self.unit, h * .34))
        return margin, bottom - height, w - 2 * margin, height

    @property
    def dock(self):
        w, h = self.size
        u = self.unit
        margin = round(12 * u)
        height = round(170 * u)
        return margin, h - round(36 * u) - height, w - 2 * margin, height

    @property
    def image(self):
        w, h = self.size
        px, py, _, _ = self.panel
        margin = round(12 * self.unit)
        area_w = (px if self.sidebar else w) - 2 * margin
        bottom = self.dock[1] - margin if self.sidebar else py - margin
        area_h = bottom - self.header - margin
        ih, iw = self.shape[:2]
        scale = min(max(1, area_w) / iw, max(1, area_h) / ih)
        dw, dh = max(1, round(iw * scale)), max(1, round(ih * scale))
        return margin + (area_w - dw) // 2, self.header + margin + (area_h - dh) // 2, dw, dh

    def camera_point(self, x, y):
        left, top, width, height = self.image
        if not (left <= x < left + width and top <= y < top + height):
            return None
        return (min(self.shape[1] - 1, int((x - left) * self.shape[1] / width)),
                min(self.shape[0] - 1, int((y - top) * self.shape[0] / height)))

    def display_point(self, x, y):
        left, top, width, height = self.image
        return left + x * width / self.shape[1], top + y * height / self.shape[0]

    @property
    def buttons(self):
        x, y, w, _ = self.dock
        u = self.unit
        gap = round(7 * u)
        top = y + round(27 * u)
        row = round(39 * u)
        split = round(w * .57)
        setup = (("REFERENCE_NEUTRAL", "Set physical neutral"),
                 ("ECHO", "Confirm echo target"), ("ARM", "Manual arm: OFF"),
                 ("AUTO", "Enable auto"))
        primary = (("GRAB", "Manual Grab"), ("NEUTRAL", "Prepare / Return Rest"), ("STOP", "STOP"))
        result = []
        for left, available, labels in ((x, split - 2 * gap, setup),
                                         (x + split, w - split, primary)):
            bw = (available - (len(labels) - 1) * gap) // len(labels)
            result += [(command, label, (left + i * (bw + gap), top, bw, row))
                       for i, (command, label) in enumerate(labels)]
        _, _, (rx, ry, rw, rh) = result[0]
        result.append(("REFERENCE_REST", "Set retracted", (rx, ry + rh + gap, rw, rh)))
        settings = (("STROKE_A", "A pulses: 900"), ("STROKE_B", "B pulses: 900"),
                    ("APPLY_STROKES", "Apply pulses"), ("JOG_A_PLUS", "A +100"),
                    ("JOG_A_MINUS", "A -100"), ("JOG_B_PLUS", "B +100"), ("JOG_B_MINUS", "B -100"))
        sw = (w - 6 * gap) // 7
        result += [(command, label, (x + i * (sw + gap), y + round(119 * u), sw, round(30 * u)))
                   for i, (command, label) in enumerate(settings)]
        return result

    def control_at(self, x, y):
        return next((command for command, _, (left, top, w, h) in self.buttons
                     if left <= x < left + w and top <= y < top + h), None)

    @property
    def status_slots(self):
        x, y, w, h = self.panel
        u = self.unit
        top = y + round(13 * u)
        margin = round(10 * u)
        if self.sidebar:
            step = (h - 2 * margin) / 6
            return [(x + margin, round(top + i * step), w - 2 * margin,
                     max(1, int((step - 22 * u) / (17 * u))))
                    for i in range(6)]
        step = (h - 2 * margin) / 3
        return [(x + margin + (i % 2) * (w // 2), round(top + (i // 2) * step),
                 w // 2 - 2 * margin, max(1, int((step - 22 * u) / (17 * u)))) for i in range(6)]


def text_lines(text, width, scale, limit=2):
    """Pixel-width wrapping with a visible ellipsis at a bounded line count."""
    text = str(text).encode('ascii', 'replace').decode('ascii')
    words = text.split()
    lines = []
    while words and len(lines) < limit:
        line = words.pop(0)
        while words and cv2.getTextSize(line + ' ' + words[0], cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] <= width:
            line += ' ' + words.pop(0)
        while cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > width and line:
            line = line[:-1]
            if not words: words = ['...']
        lines.append(line)
    if words and lines:
        line = lines[-1]
        while line and cv2.getTextSize(line + '...', cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] > width:
            line = line[:-1]
        lines[-1] = line + '...'
    return lines
