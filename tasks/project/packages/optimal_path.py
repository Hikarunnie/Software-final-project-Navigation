import heapq
import math
from tasks.project.packages.road_map import road_map


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


def dijkstra(start, goal, graph=road_map):
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
            "distance": math.inf,
        }

    edges = []
    for a, b in zip(path, path[1:]):
        shortest = graph.shortest_edge(a, b)
        if shortest is None:
            raise ValueError(f"No edge between {a} and {b}")
        edge_id, length = shortest
        edges.append(edge_id)

    return {
        "path": path,
        "edges": edges,
        "distance": distances[goal],
    }

if __name__ == "__main__":
    print("Dijkstra 1 -> 3:", dijkstra(1, 3))
    print("Dijkstra 3 -> 1:", dijkstra(3, 1))
    print("Same node:", dijkstra(3, 3))