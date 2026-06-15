import heapq
import math

from tasks.project.packages.road_map import road_map

# Compass directions in clockwise order — used for turn calculation.
_CLOCKWISE = ["N", "E", "S", "W"]


def compute_maneuver(heading, exit_dir):
    """
    Return the maneuver needed to go from current heading to exit_dir.

    heading  : current compass direction the robot is facing (N/E/S/W)
    exit_dir : compass direction the road exits the intersection (N/E/S/W)
    returns  : 'forward' | 'right' | 'turnaround' | 'left'
    """
    hi = _CLOCKWISE.index(heading)
    ei = _CLOCKWISE.index(exit_dir)
    diff = (ei - hi) % 4
    return ("forward", "right", "turnaround", "left")[diff]


def apply_maneuver(heading, maneuver):
    """
    Return the new compass heading after executing a maneuver.

    heading  : current compass direction (N/E/S/W)
    maneuver : 'forward' | 'right' | 'turnaround' | 'left'
    returns  : new compass direction (N/E/S/W)
    """
    idx = _CLOCKWISE.index(heading)
    delta = {"forward": 0, "right": 1, "turnaround": 2, "left": -1}
    return _CLOCKWISE[(idx + delta.get(maneuver, 0)) % 4]


def reconstruct_path(previous, start, goal):
    path = []
    current = goal

    while current is not None:
        path.append(current)
        if current == start:
            break
        current = previous.get(current)

    path.reverse()

    if not path or path[0] != start:
        return []

    return path


def dijkstra(start, goal, start_heading="N", graph=road_map):
    if start not in graph.nodes:
        raise ValueError(f"Start node {start} does not exist")

    if goal not in graph.nodes:
        raise ValueError(f"Goal node {goal} does not exist")

    distances = {node: math.inf for node in graph.all_nodes()}
    previous = {node: None for node in graph.all_nodes()}

    distances[start] = 0
    pq = [(0, start)]

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        if current_distance > distances[current_node]:
            continue

        if current_node == goal:
            break

        for neighbor, length, edge_id in graph.all_neighbors_shortest(current_node):
            new_distance = current_distance + length

            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous[neighbor] = current_node
                heapq.heappush(pq, (new_distance, neighbor))

    path = reconstruct_path(previous, start, goal)

    if not path:
        return {
            "path": [],
            "edges": [],
            "directions": [],
            "distance": math.inf,
        }

    # Build edge list and compute the maneuver at each intersection.
    edges = []
    directions = []
    heading = start_heading

    for a, b in zip(path, path[1:]):
        shortest = graph.shortest_edge(a, b)
        if shortest is None:
            raise ValueError(f"No edge between {a} and {b}")
        edge_id, _ = shortest
        edges.append(edge_id)

        edge_data = graph.get_edge(edge_id)
        if edge_data["from"] == a:
            exit_dir = edge_data["direction1"]
        else:
            exit_dir = edge_data["direction2"]

        maneuver = compute_maneuver(heading, exit_dir)
        directions.append(maneuver)
        heading = exit_dir  # robot's heading after traversing this edge

    return {
        "path": path,
        "edges": edges,
        "directions": directions,
        "distance": distances[goal],
    }
