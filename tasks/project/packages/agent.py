import time

from tasks.project.packages.road_map import road_map
from tasks.project.packages.optimal_path import dijkstra
import servers.project.virtual_server as server


def main(camera, wheels, leds, stop_event):
    """
    Main agent loop. Reads start/goal nodes from the server,
    computes the shortest path with Dijkstra, and drives the route.
    Called by the task launcher with camera, wheels, leds, and stop_event.
    """
    print("[Agent] Started")
    print(f"[Agent] Start: {server.current_node}, Goal: {server.goal_node}")

    try:
        while not stop_event.is_set():
            start = server.current_node
            goal = server.goal_node

            # Read camera frame — skip if not available
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.02)
                continue

            # Nothing to do if already at goal
            if start == goal:
                wheels.set_wheels_speed(0.0, 0.0)
                time.sleep(0.1)
                continue

            # Compute shortest path from current node to goal
            route = dijkstra(start, goal)
            print(f"[Agent] path={route['path']}, distance={route['distance']}")

            # Navigation state machine — waiting for teammate implementation
            # Needs: follow path segment by segment, detect intersections, turn correctly
            time.sleep(1)

    finally:
        # Always stop wheels and LEDs on exit
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()