from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}

STOP_MEMORY = 0
CLEAR_FRAMES = 0


def should_stop(detections: List[Detection], img_size: int) -> Tuple[bool, str]:
    global STOP_MEMORY, CLEAR_FRAMES

    found = None

    if detections is None:
        detections = []

    for bbox, score, class_id in detections:
        xmin, ymin, xmax, ymax = bbox

        if class_id not in [0, 1, 2]:
            continue

        if score < 0.40:
            continue

        box_height = ymax - ymin
        bottom_x = (xmin + xmax) / 2
        bottom_y = ymax

        in_front = img_size * 0.25 < bottom_x < img_size * 0.75
        close = bottom_y > img_size * 0.45
        big_enough = box_height > img_size * 0.06

        if in_front and close and big_enough:
            found = class_id
            break

    if found is not None:
        STOP_MEMORY = 35
        CLEAR_FRAMES = 0
        return True, f"Stopping for {class_names.get(found, found)}"

    if STOP_MEMORY > 0:
        STOP_MEMORY -= 1
        return True, "Keeping stop briefly"

    CLEAR_FRAMES += 1
    if CLEAR_FRAMES < 15:
        return True, "Waiting until road is clear"

    return False, ""