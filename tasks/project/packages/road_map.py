"""
road_map.py — Duckietown road network map

Represents the road network as a weighted undirected graph.
Nodes = intersections, Edges = roads between them with lengths.

Usage:
    from road_map import road_map

    # Get all neighbors of intersection 1 (returns list of (neighbor, length, edge_id))
    road_map.neighbors(1)

    # Get all edges between two intersections
    road_map.edges_between(2, 3)

    # Get a specific edge by id
    road_map.get_edge("1-2-a")
"""

# Important robot always starts at intersection and ends at intersection, so start and endpoints are always nodes
class RoadMap:
    def __init__(self):
        # Intersections (nodes)
        # Coordinates only needed with A* since we not use it no need to fill them in
        self.nodes = {
            1: {"id": 1, "x": 0.0, "y": 0.0},
            2: {"id": 2, "x": 0.0, "y": 0.0},
            3: {"id": 3, "x": 0.0, "y": 0.0},
        }

        # Roads (edges)
        # Each road has a unique id, the two intersections it connects, and its length.
        # Multiple roads can connect the same two intersections.
        self.edges = {
            "1-2-a": {"from": 1, "to": 2, "length": 1},
            "1-2-b": {"from": 1, "to": 2, "length": 4},
            "1-3-a": {"from": 1, "to": 3, "length": 3},
            "2-3-a": {"from": 2, "to": 3, "length": 2},
            "2-3-b": {"from": 2, "to": 3, "length": 5},
        }

    def neighbors(self, node_id):
        """
        Return all roads reachable from node_id.
        Returns a list of (neighbor_id, length, edge_id) tuples.
        """
        result = []
        for edge_id, edge in self.edges.items():
            if edge["from"] == node_id:
                result.append((edge["to"], edge["length"], edge_id))
            elif edge["to"] == node_id:
                result.append((edge["from"], edge["length"], edge_id))
        return result

    def edges_between(self, node_a, node_b):
        """
        Return all edges between node_a and node_b (there may be more than one road).
        Returns a list of (edge_id, length) tuples, sorted by length ascending.
        """
        result = []
        for edge_id, edge in self.edges.items():
            if (edge["from"] == node_a and edge["to"] == node_b) or \
               (edge["from"] == node_b and edge["to"] == node_a):
                result.append((edge_id, edge["length"]))
        return sorted(result, key=lambda x: x[1])

    def shortest_edge(self, node_a, node_b):
        """
        Return the shortest road between node_a and node_b.
        Returns (edge_id, length) or None if no road exists.
        """
        edges = self.edges_between(node_a, node_b)
        return edges[0] if edges else None

    def get_node(self, node_id):
        """Return node data dict for node_id."""
        return self.nodes.get(node_id)

    def get_edge(self, edge_id):
        """Return edge data dict for edge_id."""
        return self.edges.get(edge_id)

    def all_nodes(self):
        """Return list of all node ids."""
        return list(self.nodes.keys())

    def all_edges(self):
        """Return list of all edge ids."""
        return list(self.edges.keys())


# Singleton — import this in your pathfinding and navigation code
road_map = RoadMap()


# Quick sanity check when run directly
if __name__ == "__main__":
    print("Nodes:", road_map.all_nodes())
    print("Edges:")
    for eid, e in road_map.edges.items():
        print(f"  {eid}: {e['from']} <-> {e['to']}, length={e['length']}")
    print()
    print("Neighbors of 1:", road_map.neighbors(1))
    print("Neighbors of 2:", road_map.neighbors(2))
    print("Roads between 1 and 2:", road_map.edges_between(1, 2))
    print("Roads between 2 and 3:", road_map.edges_between(2, 3))
    print("Shortest 1->2:", road_map.shortest_edge(1, 2))