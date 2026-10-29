import cv2
import numpy as np


def annotate(rgb, pegs, sockets=None):
    output = rgb.copy()
    for obj in [*pegs.values(), *(sockets or {}).values()]:
        cv2.drawContours(output, [obj.contour], -1, (255, 255, 255), 2)
        u, v = map(round, obj.centroid)
        x, y, w, h = cv2.boundingRect(obj.contour)
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 255), 1)
        label = f"{obj.color} {obj.shape} ({obj.position[0]:.3f},{obj.position[1]:.3f},{obj.position[2]:.3f})"
        # A dark label background stays readable over white board surfaces.
        font, scale = cv2.FONT_HERSHEY_SIMPLEX, .48
        (tw, th), _ = cv2.getTextSize(label, font, scale, 1)
        tx = min(max(2, u - 80), max(2, output.shape[1] - tw - 4))
        ty = max(th + 4, v - 20)
        cv2.rectangle(output, (tx - 2, ty - th - 3), (tx + tw + 2, ty + 3), (20, 24, 30), -1)
        cv2.putText(output, label, (tx, ty), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def depth_visual(depth):
    normalized = np.clip((depth - .6) / 1.0 * 255, 0, 255).astype(np.uint8)
    return cv2.cvtColor(cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
