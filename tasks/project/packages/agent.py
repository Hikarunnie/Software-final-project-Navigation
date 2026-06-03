import time
import cv2
import numpy as np

from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra
from tasks.project.packages.red_line_detection import detect_red_line
from tasks.visual_lane_servoing.packages.agent import LaneServoingAgent
import servers.project.virtual_server as server

_TURN_DURATION  = 1.5   # seconds to execute a turn — tune this
_TURN_SPEED     = 0.4
_STRAIGHT_DURATION = 0.5  # extra forward time after stopping before turning

# States
_LANE_FOLLOWING = 'LANE_FOLLOWING'
_STOPPING       = 'STOPPING'
_GOAL_REACHED   = 'GOAL_REACHED'

# How many consecutive frames must see red line before we believe it
_RED_LINE_CONFIRM_FRAMES = 3


def _turn_direction(prev_node: int, curr_node: int, next_node: int) -> str:
    """
    Determine turn direction using heading change.
    prev -> curr gives the incoming direction.
    curr -> next gives the outgoing direction.
    Cross product tells us left/right/straight.
    """
    prev = road_map.get_node(prev_node)
    curr = road_map.get_node(curr_node)
    nxt  = road_map.get_node(next_node)

    if prev is None or curr is None or nxt is None:
        return 'straight'

    # Incoming vector
    dx1 = curr['x'] - prev['x']
    dy1 = curr['y'] - prev['y']

    # Outgoing vector
    dx2 = nxt['x'] - curr['x']
    dy2 = nxt['y'] - curr['y']

    # Cross product: positive = left turn, negative = right turn
    cross = dx1 * dy2 - dy1 * dx2

    dot = dx1 * dx2 + dy1 * dy2

    if abs(cross) < 0.01:
        return 'straight'
    elif cross > 0:
        return 'left'
    else:
        return 'right'


def _execute_turn(direction: str, wheels, stop_ev):
    """Drive the turn maneuver for a fixed duration."""
    print(f"[Nav] Turning {direction}")

    # First move forward a bit to clear the red line
    wheels.set_wheels_speed(0.3, 0.3)
    time.sleep(_STRAIGHT_DURATION)

    if direction == 'left':
        left, right = -_TURN_SPEED, _TURN_SPEED
    elif direction == 'right':
        left, right = _TURN_SPEED, -_TURN_SPEED
    else:
        left, right = _TURN_SPEED, _TURN_SPEED

    end_time = time.time() + _TURN_DURATION
    while time.time() < end_time and not stop_ev.is_set():
        wheels.set_wheels_speed(left, right)
        time.sleep(0.05)

    wheels.set_wheels_speed(0.0, 0.0)
    time.sleep(0.2)


def main(camera, wheels, leds, stop_event):
    """
    Navigation agent state machine.
    States: LANE_FOLLOWING -> STOPPING -> (turn) -> LANE_FOLLOWING -> ... -> GOAL_REACHED
    """
    print("[Nav] Started")

    lane_agent         = LaneServoingAgent()
    state              = _LANE_FOLLOWING
    route              = None
    path_index         = 0
    red_line_counter   = 0  # consecutive frames with red line

    try:
        while not stop_event.is_set():

            start = server.current_node
            goal  = server.goal_node

            ok, frame_rgb = camera.read()
            if not ok or frame_rgb is None:
                time.sleep(0.02)
                continue

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # Recompute route if not set or start/goal changed
            if route is None or route['path'][0] != start or route['path'][-1] != goal:
                if start == goal:
                    state = _GOAL_REACHED
                else:
                    route            = dijkstra(start, goal)
                    path_index       = 0
                    red_line_counter = 0
                    state            = _LANE_FOLLOWING
                    print(f"[Nav] New route: {route['path']}")

            # ── GOAL REACHED ──────────────────────────────────────────────
            if state == _GOAL_REACHED:
                wheels.set_wheels_speed(0.0, 0.0)
                print("[Nav] Goal reached! Doing victory dance...")
                # Import dance from virtual server and do victory dance
                from servers.project.virtual_server import dance, maneuver_stop
                dance(3.0, maneuver_stop)
                time.sleep(0.5)
                continue

            if route is None or not route['path']:
                wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(0.1)
                continue

            # ── LANE FOLLOWING ────────────────────────────────────────────
            if state == _LANE_FOLLOWING:
                left, right = lane_agent.compute_commands(frame_rgb)
                wheels.set_wheels_speed(left, right)

                # Confirm red line over multiple frames to avoid false positives
                if detect_red_line(frame_bgr):
                    red_line_counter += 1
                    print(f"[Nav] Red line frame {red_line_counter}/{_RED_LINE_CONFIRM_FRAMES}")
                else:
                    red_line_counter = 0

                if red_line_counter >= _RED_LINE_CONFIRM_FRAMES:
                    red_line_counter = 0
                    print(f"[Nav] Red line confirmed at path index {path_index}")
                    state = _STOPPING

            # ── STOPPING ──────────────────────────────────────────────────
            elif state == _STOPPING:
                wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(0.3)

                # Advance path index
                path_index += 1

                if path_index >= len(route['path']):
                    server.current_node = route['path'][-1]
                    state = _GOAL_REACHED
                    continue

                # Update current node on server
                server.current_node = route['path'][path_index]
                print(f"[Nav] Now at node {server.current_node}")

                # Check if this is the last node
                if path_index >= len(route['path']) - 1:
                    state = _GOAL_REACHED
                    continue

                # Determine turn direction using prev -> curr -> next
                prev_node = route['path'][path_index - 1]
                curr_node = route['path'][path_index]
                next_node = route['path'][path_index + 1]

                direction = _turn_direction(prev_node, curr_node, next_node)
                print(f"[Nav] Turn: {prev_node} -> {curr_node} -> {next_node} = {direction}")
                _execute_turn(direction, wheels, stop_event)
                state = _LANE_FOLLOWING

    finally:
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()
        print("[Nav] Stopped")