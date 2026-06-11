from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}


def should_stop(detections: List[Detection], img_size: int) -> Tuple[bool, str]:
    for bbox, score, class_id in detections:
        xmin, ymin, xmax, ymax = bbox
        if class_id in class_names:
            return True, f"Detected {class_names.get(class_id, 'object')} nearby!"
    return False, ""
