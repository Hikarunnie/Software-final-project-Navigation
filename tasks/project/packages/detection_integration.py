from typing import Tuple

# Path to the trained model weights (.onnx file).
# Relative paths resolve from the project root.
MODEL_PATH = "tasks/project/models/best.onnx"


def NUMBER_FRAMES_SKIPPED() -> int:
    # Higher = run inference less often (cheaper).
    return 1


def filter_by_classes(pred_class: int) -> bool:
    """Return False to drop this prediction."""
    return pred_class == 0


def filter_by_scores(score: float) -> bool:
    """Confidence in [0.0, 1.0]. Return False to drop low-confidence boxes."""
    return score >= 0.5


def filter_by_bboxes(bbox: Tuple[int, int, int, int]) -> bool:
    """bbox is (xmin, ymin, xmax, ymax) in pixels. Return False to drop."""
    xmin, ymin, xmax, ymax = bbox
    width = xmax - xmin
    height = ymax - ymin
    return (width * height) > 800
