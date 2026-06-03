import json
from typing import List

CLASSES = ['duckie', 'truck', 'sign']
IMAGE_SIZE = 416


def convert_labelme_json(json_path: str, img_w: int, img_h: int) -> List[str]:
    if json_path is None:
        return []

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    lines = []

    for shape in data.get("shapes", []):
        label = shape.get("label")

        if label not in CLASSES:
            continue

        points = shape.get("points", [])
        if len(points) < 2:
            continue

        cls_id = CLASSES.index(label)

        x1, y1 = points[0]
        x2, y2 = points[1]

        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)

        xmin_s = xmin * IMAGE_SIZE / img_w
        xmax_s = xmax * IMAGE_SIZE / img_w
        ymin_s = ymin * IMAGE_SIZE / img_h
        ymax_s = ymax * IMAGE_SIZE / img_h

        cx = (xmin_s + xmax_s) / 2 / IMAGE_SIZE
        cy = (ymin_s + ymax_s) / 2 / IMAGE_SIZE
        w = (xmax_s - xmin_s) / IMAGE_SIZE
        h = (ymax_s - ymin_s) / IMAGE_SIZE

        lines.append(f"{cls_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    return lines