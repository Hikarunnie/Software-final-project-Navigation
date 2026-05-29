import time
from road_map import road_map  # your map file
from dance import Dance

def main(camera, wheels, leds, stop_event):
    start = 1
    goal = 3

    dancer = Dance(wheels, leds)

    print(f"Start: {start}, Goal: {goal}")
    print(f"Neighbors of start: {road_map.neighbors(start)}")
    print(f"Shortest path from {start} to {goal}: {road_map.shortest_edge(start, goal)}")

    try:
        while not stop_event.is_set():
            ok, frame = camera.read()
            if not ok:
                time.sleep(0.02)
                continue

            # When goal is reached:
            # dancer.perform()
            # break

    finally:
        wheels.set_wheels_speed(0.0, 0.0)
        if leds:
            leds.all_off()