"""Extract foreground regions and track one selected object."""

from dataclasses import dataclass
from math import hypot

import cv2
import numpy as np


@dataclass(eq=False)
class Region:
    box: tuple[int, int, int, int]
    pixels: int
    center: tuple[float, float]
    outline: np.ndarray


def find_regions(frame, threshold=180, min_pixels=80, dark=False, roi=None):
    """Threshold a region and return the mask and boxes in camera coordinates."""
    height, width = frame.shape[:2]
    x, y, w, h = roi if roi is not None else (0, 0, width, height)
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
        raise ValueError("The bench region must be inside the camera image.")
    gray = cv2.cvtColor(frame[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY)
    mode = cv2.THRESH_BINARY_INV if dark else cv2.THRESH_BINARY
    _, local_mask = cv2.threshold(gray, threshold, 255, mode)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[y:y + h, x:x + w] = local_mask
    return mask, regions_from_mask(mask, min_pixels, (x, y, w, h))


def regions_from_mask(mask, min_pixels=80, roi=None):
    """Extract regions without changing or filling the supplied foreground mask."""
    height, width = mask.shape
    x, y, w, h = roi if roi is not None else (0, 0, width, height)
    local_mask = mask[y:y + h, x:x + w]
    count, labels, stats, centers = cv2.connectedComponentsWithStats(local_mask, connectivity=8)
    regions = []
    for label in range(1, count):
        left, top, box_w, box_h, pixels = map(int, stats[label])
        # Reject noise and an almost entirely foreground bench region.
        if pixels < min_pixels or pixels >= 0.9 * w * h:
            continue
        component = (labels[top:top + box_h, left:left + box_w] == label).astype(np.uint8) * 255
        outlines, _ = cv2.findContours(
            component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x + left, y + top)
        )
        regions.append(Region(
            box=(x + left, y + top, box_w, box_h),
            pixels=pixels,
            center=(float(centers[label, 0] + x), float(centers[label, 1] + y)),
            outline=max(outlines, key=cv2.contourArea),
        ))
    return sorted(regions, key=lambda region: region.box[:2])


def overlap(first, second):
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    intersection = max(0, min(ax + aw, bx + bw) - max(ax, bx))
    intersection *= max(0, min(ay + ah, by + bh) - max(ay, by))
    union = aw * ah + bw * bh - intersection
    return intersection / union if union else 0.0


def movement_limit(box):
    """Maximum frame-to-frame movement in pixels for guided tracking."""
    return max(48.0, 1.5 * hypot(box[2], box[3]))


class Selection:
    """Follow a unique region until the operator needs to select again."""

    def __init__(self):
        self.number = 0
        self.clear()

    def clear(self, reason="Click one box to select an unknown object"):
        self.current = None
        self.state = "unselected"
        self.reason = reason

    def choose(self, point, regions):
        px, py = point
        matches = [
            region for region in regions
            if region.box[0] <= px < region.box[0] + region.box[2]
            and region.box[1] <= py < region.box[1] + region.box[3]
        ]
        self.current = None
        if len(matches) == 1:
            self.number += 1
            self.current = matches[0]
            self.state = "tracking"
            self.reason = "Selected foreground region"
        elif matches:
            self.state = "ambiguous"
            self.reason = "Overlapping boxes; click an unambiguous part or separate objects"
        else:
            self.clear("No candidate at that point; adjust the mask or bench region")

    def matches(self, regions, *, moving=False):
        if self.state != "tracking":
            return []
        previous = self.current
        matches = []
        for region in regions:
            ratio = region.pixels / previous.pixels
            distance = hypot(region.center[0] - previous.center[0], region.center[1] - previous.center[1])
            # The wider search can follow a box that no longer overlaps.
            limit = movement_limit(previous.box) if moving else max(24.0, 0.6 * hypot(previous.box[2], previous.box[3]))
            shape_ok = not moving or all(0.65 <= region.box[i] / previous.box[i] <= 1.5 for i in (2, 3))
            if (0.6 <= ratio <= 1.6 and distance <= limit and shape_ok
                    and (moving or overlap(previous.box, region.box) >= 0.1)):
                matches.append(region)
        return matches

    def update(self, regions, *, moving=False):
        if self.state != "tracking":
            return
        matches = self.matches(regions, moving=moving)
        if len(matches) == 1:
            self.current = matches[0]
        else:
            self.current = None
            self.state = "ambiguous" if matches else "lost"
            self.reason = "Target uncertain; click again" if matches else "Target lost; click again to reacquire"
