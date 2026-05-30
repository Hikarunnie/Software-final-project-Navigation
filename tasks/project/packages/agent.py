import time
from road_map import road_map
import servers.project.virtual_server as server

def main(camera, wheels, leds, stop_event):

    print(f"[Agent] Start: {server.current_node}, Goal: {server.goal_node}")
    print(f"[Agent] Neighbors of start: {road_map.neighbors(server.current_node)}")
    print(f"[Agent] Shortest edge from {server.current_node} to {server.goal_node}: {road_map.shortest_edge(server.current_node, server.goal_node)}")

    try:
        while not stop_event.is_set():
            # These are read fresh every loop iteration
            start = server.current_node
            goal = server.goal_node

            ok, frame = camera.read()
            if not ok:
                time.sleep(0.02)
                continue

            # pathfinding code will go here,
            # using start and goal which update live from the UI

    finally:
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()