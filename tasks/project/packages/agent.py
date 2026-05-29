import time
from road_map import road_map  # your map file

def main(camera, wheels, leds, stop_event):
    # Define start and goal
    start = 1
    goal = 3

    # Print map
    print(f"Start: {start}, Goal: {goal}")
    print(f"Neighbors of start: {road_map.neighbors(start)}")
    print(f"Shortest path from {start} to {goal}: {road_map.shortest_edge(start, goal)}")

    try:
        while not stop_event.is_set():
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.02)
                continue

            # pathfinding code will go here,
            # using road_map to find the path

    finally:
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()