from typing import Tuple

MODEL_PATH = "tasks/object_detection/models/best.onnx"


def NUMBER_FRAMES_SKIPPED() -> int:
    # detect every frame
    return 0


def filter_by_classes(pred_class: int) -> bool:
    # allow trained classes: duckie, truck, sign
    return pred_class in [0, 1, 2]


def filter_by_scores(score: float) -> bool:
    # not too strict
    return score >= 0.45


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    xmin, ymin, xmax, ymax = bbox

    w = xmax - xmin
    h = ymax - ymin

    if w <= 0 or h <= 0:
        return False

    # do NOT use area 800, it can remove far but valid objects
    return True