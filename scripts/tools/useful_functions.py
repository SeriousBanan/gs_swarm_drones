"""
Module with useful functions for program.
"""

from math import sqrt

from .objects import Vertex, Graph, Coords


def initialize_graph_from_dict(field: dict, altitude: float) -> Graph:
    """
    Initializing graph from dict with
    list of vertexes and list of edges.
    """

    graph = Graph()

    for vertex_data in field["vertexes"]:
        vertex = Vertex(id_=vertex_data["id"],
                        coords=Coords(x=vertex_data["x"], y=vertex_data["y"], z=altitude))

        graph.add_vertex(vertex)

    for edge_data in field["edges"]:
        from_vertex = graph[edge_data["from id"]]
        to_vertex = graph[edge_data["to id"]]

        from_vertex.add_neighbor(to_vertex)

    return graph


def distance(from_vertex: Vertex, to_vertex: Vertex) -> float:
    """Calculate distance between two vertexes."""
    from_coord = from_vertex.coords
    to_coord = to_vertex.coords

    return sqrt((to_coord.x - from_coord.x) ** 2 +
                (to_coord.y - from_coord.y) ** 2 +
                (to_coord.z - from_coord.z) ** 2)
