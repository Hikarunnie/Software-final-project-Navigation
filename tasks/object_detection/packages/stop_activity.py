from typing import List, Tuple

Detection = Tuple[Tuple[int, int, int, int], float, int]

class_names = {0: 'duckie', 1: 'truck', 2: 'sign'}

LANE_FOLLOWING = 'LANE_FOLLOWING'
OBSTACLE_PRESENT = 'OBSTACLE_PRESENT'
DECELERATING = 'DECELERATING'

_state = LANE_FOLLOWING
_decel_frame = 0
_clear_count = 0
_stop_reason = ''
_stop_confirm = 0  # consecutive frames meeting the stop condition


def _get_best_detection(detections, img_size):
    best = None
    best_bottom = -1
    for bbox, score, cls_id in detections:
        _, _, _, y2 = bbox
        if y2 > best_bottom:
            best_bottom = y2
            best = (bbox, score, cls_id)
    return best


def _is_in_opposing_lane(bbox, img_size) -> bool:
    x1, _, x2, _ = bbox
    cx = (x1 + x2) / 2
    lane_center = img_size * 0.35
    return cx < lane_center


def should_stop(detections, img_size):
    global _state, _decel_frame, _clear_count, _stop_reason, _stop_confirm

    RELEASE_FRAMES = 5
    STOP_Y = 0.70
    WARN_Y = 0.55
    STOP_CONFIRM_FRAMES = 3  # require N consecutive same-lane frames before hard-stopping

    filtered = [d for d in detections if not _is_in_opposing_lane(d[0], img_size)]

    if _state == LANE_FOLLOWING:
        if not filtered:
            _stop_confirm = 0
            return False, ''

        best = _get_best_detection(filtered, img_size)
        _, _, _, y2 = best[0]
        cls_id = best[2]

        if y2 > STOP_Y * img_size:
            _stop_confirm += 1
            _stop_reason = f'slowing_for_{class_names.get(cls_id, str(cls_id))}'
            if _stop_confirm >= STOP_CONFIRM_FRAMES:
                _state = OBSTACLE_PRESENT
                _stop_confirm = 0
                _stop_reason = f'{class_names.get(cls_id, str(cls_id))}_ahead'
                return True, _stop_reason
            return False, _stop_reason

        _stop_confirm = 0

        if y2 > WARN_Y * img_size:
            _state = DECELERATING
            _decel_frame = 0
            _stop_reason = f'slowing_for_{class_names.get(cls_id, str(cls_id))}'
            return False, _stop_reason

        return False, ''

    elif _state == DECELERATING:
        if not filtered:
            _state = LANE_FOLLOWING
            _decel_frame = 0
            _stop_confirm = 0
            return False, ''

        _decel_frame += 1
        best = _get_best_detection(filtered, img_size)
        _, _, _, y2 = best[0]
        cls_id = best[2]

        if y2 > STOP_Y * img_size:
            _stop_confirm += 1
            if _stop_confirm >= STOP_CONFIRM_FRAMES:
                _state = OBSTACLE_PRESENT
                _stop_confirm = 0
                _stop_reason = f'{class_names.get(cls_id, str(cls_id))}_ahead'
                return True, _stop_reason
            return False, _stop_reason

        _stop_confirm = 0
        return False, _stop_reason

    else:  # OBSTACLE_PRESENT
        if not filtered:
            _clear_count += 1
            if _clear_count >= RELEASE_FRAMES:
                _state = LANE_FOLLOWING
                _clear_count = 0
                _stop_confirm = 0
                return False, ''
            return True, _stop_reason

        _clear_count = 0
        return True, _stop_reason