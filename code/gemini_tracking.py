"""Track a confirmed Gemini box using its visible foreground anchor."""

from dataclasses import dataclass

from contour_tracking import Region, Selection


@dataclass(eq=False)
class AnchoredRegion(Region):
    anchor: Region


class ConfirmedBoxGuide:
    """Project the confirmed box from the anchor's position and size."""

    def __init__(self, guide, box, anchor):
        self.guide, self.box, self.anchor = guide, box, anchor

    def mask(self):
        return self.guide.mask()

    def project(self, region):
        x, y, w, h = region.box
        ax, ay, aw, ah = self.anchor.box
        bx, by, bw, bh = self.box
        sx, sy = w / aw, h / ah
        # Reject abrupt changes in anchor scale or aspect ratio.
        if not (0.5 <= sx <= 2 and 0.5 <= sy <= 2 and min(sx, sy) / max(sx, sy) >= 0.7):
            return None
        left, top = round(x + (bx - ax) * sx), round(y + (by - ay) * sy)
        right, bottom = left + round(bw * sx), top + round(bh * sy)
        height, width = self.guide.shape
        # Tolerate at most two pixels of projection rounding at the image edge.
        if left < -2 or top < -2 or right > width + 2 or bottom > height + 2:
            return None
        left, top, right, bottom = max(0, left), max(0, top), min(width, right), min(height, bottom)
        return AnchoredRegion((left, top, right - left, bottom - top), region.pixels,
                              ((left + right) / 2, (top + bottom) / 2), region.outline, region)

    def _anchor_box(self, box):
        if box is None:
            return self.anchor.box
        x, y, w, h = box
        bx, by, bw, bh = self.box
        ax, ay, aw, ah = self.anchor.box
        return (round(x + (ax - bx) * w / bw), round(y + (ay - by) * h / bh),
                max(1, round(aw * w / bw)), max(1, round(ah * h / bh)))

    def detect(self, frame, box=None, min_pixels=20):
        mask, regions = self.guide.detect(frame, self._anchor_box(box), min_pixels)
        return mask, [projected for region in regions if (projected := self.project(region)) is not None]

    def follow(self, frame, selection, box=None, min_pixels=20):
        # Match the segmented anchor before projecting the estimated box.
        local = Selection()
        local.state, local.reason, local.number = selection.state, selection.reason, selection.number
        local.current = selection.current.anchor if isinstance(selection.current, AnchoredRegion) else None
        if local.state == "tracking" and local.current is None:
            local.state, local.reason = "lost", "Target anchor lost; select again"
        mask, regions = self.guide.follow(frame, local, self._anchor_box(box), min_pixels)
        projected = [item for region in regions if (item := self.project(region)) is not None]
        selection.state, selection.reason = local.state, local.reason
        selection.current = next((item for item in projected if item.anchor is local.current), None)
        if local.current is not None and selection.current is None:
            selection.state, selection.reason = "lost", "Target box left the image or changed shape; select again"
        return mask, projected
