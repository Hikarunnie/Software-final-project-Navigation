import godot.utils.map


class RoadMap:
    """
    Weighted undirected graph representing the Duckietown road network.
    Nodes = intersections (curve/cross tiles), Edges = roads between them.
    Loaded automatically from the exported Godot scene.
    """

    def __init__(self, scene_name="test1_actual_map_kiu"):
        # Parse nodes and edges directly from the Godot .tscn map file
        self.nodes, self.edges = godot.utils.map.get_nodes_and_edges(scene_name)

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
        Use this for Dijkstra — ignores longer parallel roads to same node.
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


# Singleton this is needed to import in pathfinding and navigation code
road_map = RoadMap()