import time
from road_map import road_map
import servers.project.virtual_server as server
from tasks.project.packages.optimal_path import dijkstra

def main(camera, wheels, leds, stop_event):
    print("AGENT FILE LOADED")
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

            route = dijkstra(start, goal)

            print(f"[PathPlanner] start={start}, goal={goal}")
            print(f"[PathPlanner] path={route['path']}")
            print(f"[PathPlanner] edges={route['edges']}")
            print(f"[PathPlanner] distance={route['distance']}")

            time.sleep(1)

    finally:
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()