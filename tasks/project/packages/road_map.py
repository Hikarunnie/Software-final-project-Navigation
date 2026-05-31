import godot.utils.map


class RoadMap:
    """
    Weighted undirected graph representing the Duckietown road network.
    Nodes = intersections (curve/cross tiles), Edges = roads between them.
    In simulation: loaded from the Godot scene file.
    On real robot: falls back to hardcoded map matching the physical track.
    """

    def __init__(self, scene_name="test1_actual_map_kiu"):
        try:
            self.nodes, self.edges = godot.utils.map.get_nodes_and_edges(scene_name)
            print(f"[RoadMap] Loaded from scene: {scene_name} ({len(self.nodes)} nodes, {len(self.edges)} edges)")
        except Exception as e:
            print(f"[RoadMap] Could not load scene '{scene_name}': {e}")
            print("[RoadMap] Using hardcoded fallback map")
            self.nodes = {
                1:  {"id": 1,  "x": 0.9, "y": 2.1},
                2:  {"id": 2,  "x": 2.7, "y": 2.1},
                3:  {"id": 3,  "x": 2.7, "y": 3.3},
                4:  {"id": 4,  "x": 2.1, "y": 3.3},
                5:  {"id": 5,  "x": 0.9, "y": 4.5},
                6:  {"id": 6,  "x": 2.1, "y": 4.5},
                7:  {"id": 7,  "x": 0.9, "y": 6.9},
                8:  {"id": 8,  "x": 2.1, "y": 6.9},
                9:  {"id": 9,  "x": 4.5, "y": 4.5},
                10: {"id": 10, "x": 4.5, "y": 2.7},
                11: {"id": 11, "x": 5.1, "y": 2.7},
                12: {"id": 12, "x": 5.1, "y": 2.1},
            }
            self.edges = {
                "1-2-a":   {"from": 1,  "to": 2,  "length": 3},
                "1-5-a":   {"from": 1,  "to": 5,  "length": 4},
                "2-12-a":  {"from": 2,  "to": 12, "length": 4},
                "2-3-a":   {"from": 2,  "to": 3,  "length": 2},
                "3-4-a":   {"from": 3,  "to": 4,  "length": 1},
                "4-6-a":   {"from": 4,  "to": 6,  "length": 2},
                "5-6-a":   {"from": 5,  "to": 6,  "length": 2},
                "5-7-a":   {"from": 5,  "to": 7,  "length": 4},
                "6-9-a":   {"from": 6,  "to": 9,  "length": 4},
                "6-8-a":   {"from": 6,  "to": 8,  "length": 4},
                "7-8-a":   {"from": 7,  "to": 8,  "length": 2},
                "9-10-a":  {"from": 9,  "to": 10, "length": 3},
                "10-11-a": {"from": 10, "to": 11, "length": 1},
                "10-12-a": {"from": 10, "to": 12, "length": 2},
                "11-12-a": {"from": 11, "to": 12, "length": 1},
            }

    def neighbors(self, node_id):
        """Return all roads reachable from node_id as (neighbor_id, length, edge_id) tuples."""
        result = []
        for edge_id, edge in self.edges.items():
            if edge["from"] == node_id:
                result.append((edge["to"], edge["length"], edge_id))
            elif edge["to"] == node_id:
                result.append((edge["from"], edge["length"], edge_id))
        return result

    def all_neighbors_shortest(self, node_id):
        """
        Like neighbors() but returns only the shortest road to each neighbor.
        Use this for Dijkstra — ignores longer parallel roads to the same node.
        """
        seen = {}
        for neighbor, length, edge_id in self.neighbors(node_id):
            if neighbor not in seen or length < seen[neighbor][0]:
                seen[neighbor] = (length, edge_id)
        return [(neighbor, length, edge_id) for neighbor, (length, edge_id) in seen.items()]

    def edges_between(self, node_a, node_b):
        """Return all edges between node_a and node_b, sorted by length ascending."""
        result = []
        for edge_id, edge in self.edges.items():
            if (edge["from"] == node_a and edge["to"] == node_b) or \
               (edge["from"] == node_b and edge["to"] == node_a):
                result.append((edge_id, edge["length"]))
        return sorted(result, key=lambda x: x[1])

    def shortest_edge(self, node_a, node_b):
        """Return the shortest (edge_id, length) between two nodes, or None if no road exists."""
        edges = self.edges_between(node_a, node_b)
        return edges[0] if edges else None

    def get_node(self, node_id):
        """Return node data dict {id, x, y} for node_id."""
        return self.nodes.get(node_id)

    def get_edge(self, edge_id):
        """Return edge data dict {from, to, length} for edge_id."""
        return self.edges.get(edge_id)

    def all_nodes(self):
        """Return list of all node ids."""
        return list(self.nodes.keys())

    def all_edges(self):
        """Return list of all edge ids."""
        return list(self.edges.keys())


# Singleton needed to import this in pathfinding and navigation code
road_map = RoadMap()