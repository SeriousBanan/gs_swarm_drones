#!/usr/bin/env python3

"""Main module of monitoring for swarm of drones."""

import os
import json
from typing import List, Set

import rospy

PKG_PATH = os.getenv("PKG_PATH")
if PKG_PATH:
    os.chdir(PKG_PATH)

try:
    from .tools.useful_functions import (initialize_graph_from_dict,
                                         distance)
    from .tools.objects import Drone, Graph, Vertex
    from .tools.setup_loggers import logger
except ImportError:
    from tools.useful_functions import (initialize_graph_from_dict,  # type: ignore
                                        distance)  # type: ignore
    from tools.objects import Drone, Graph, Vertex  # type: ignore
    from tools.setup_loggers import logger  # type: ignore


def fill_graph_values(graph: Graph, start_vertex: Vertex) -> None:
    """Fill values of vertexes in graph."""

    # Check given attributes.
    if start_vertex not in graph:
        raise ValueError("Given vertex not in graph.")

    # Value of the current vertex equal 0.
    start_vertex.value = 0
    need_to_look = [start_vertex]

    # Bypassing through all vertexes in width.
    while need_to_look:
        vertex = need_to_look.pop(0)

        for neighbor in vertex.adjacent:
            new_value = vertex.value + distance(vertex, neighbor)

            # Save only smallest value.
            if neighbor.value == -1 or neighbor.value > new_value:
                neighbor.value = new_value
                need_to_look.append(neighbor)


def fill_global_values(drone: Drone) -> None:
    """Fill values of vertexes in global graph"""

    global_graph = drone.global_graph

    # Set all values in the graph to multiply them later.
    for vertex in global_graph:
        vertex.value = 1

    # Calculate values in each drone graph and multiply them.
    for drone_id in range(CONFIGURATION["drones count"]):
        drone_graph = global_graph.deepcopy(with_arcs=False, with_values=False)

        fill_graph_values(graph=drone_graph,
                          start_vertex=drone_graph[CONFIGURATION["drones initial positions"][str(drone_id)]])

        for vertex in drone_graph:
            global_vertex = global_graph[vertex.id_]
            global_vertex.value *= vertex.value


def split_graph(drone: Drone) -> None:
    """Split global graph to subgraphs."""

    not_chosen_vertexes = set(drone.global_graph)
    drones_possible_chooses = {}

    # Initialize drone graph and add his position to it.
    drone.graph = Graph()
    global_graph = drone.global_graph

    vertex = drone.position
    new_vertex = vertex.copy()

    drone.position = new_vertex
    drone.graph.add_vertex(new_vertex)

    # Set each drone possible chooses.
    for drone_id in range(CONFIGURATION["drones count"]):
        vertex = global_graph[CONFIGURATION["drones initial positions"][str(drone_id)]]

        not_chosen_vertexes.remove(vertex)
        drones_possible_chooses[drone_id] = vertex.adjacent.copy()

    # Distribute all vertexes between drones.
    while not_chosen_vertexes:
        # Drones chooses one by one.
        for drone_id in range(CONFIGURATION["drones count"]):
            could_choose = drones_possible_chooses[drone_id]
            could_choose.intersection_update(not_chosen_vertexes)

            if not could_choose:
                continue

            # Choose vertex with highest value.
            # If there many vertexes with max value choose that have less id.
            chosen_vertex = max(could_choose, key=lambda vertex: (vertex.value, -vertex.id_))
            not_chosen_vertexes.remove(chosen_vertex)

            # If choose current drone add vertex to it's graph.
            if drone_id == drone.id_:
                new_vertex = chosen_vertex.copy()
                drone.graph.add_vertex(new_vertex)

            # Look to the neighbor of chosen vertex and add it to possible choose
            # or create edge between previous added vertex.
            for neighbor in chosen_vertex.adjacent:
                if neighbor in not_chosen_vertexes:
                    could_choose.add(neighbor)

                elif drone_id == drone.id_ and neighbor.id_ in drone.graph:
                    new_vertex.add_neighbor(drone.graph[neighbor.id_])


def drop_arcs(graph: Graph) -> None:
    """Delete all arcs in graph."""

    for vertex in graph:
        vertex.arcs = set()


def calculate_arcs(graph: Graph, start_vertex: Vertex, not_checked: Set[Vertex]) -> None:
    """Go through vertexes in depth and calculate the weights of arcs."""

    # Check given attributes.
    if start_vertex not in graph:
        raise ValueError("Given vertex not in graph.")

    # Drop all arcs between vertexes before creating new.
    drop_arcs(graph)

    # Path needed for walking in depth and not make cycles.
    path = {start_vertex}

    def calculate_arc_weight(from_vertex: Vertex, to_vertex: Vertex) -> float:
        """Calculate weight of arc from one vertex to another."""

        path.add(to_vertex)

        # Select vertexes to which we can move and calculate arcs to them.
        for neighbor in to_vertex.adjacent:
            if (neighbor not in path and
                    neighbor.value >= to_vertex.value and
                    neighbor not in set(map(lambda arc: arc.to_vertex, to_vertex.arcs)) and
                    to_vertex not in set(map(lambda arc: arc.to_vertex, neighbor.arcs))):
                weight = calculate_arc_weight(to_vertex, neighbor)
                to_vertex.add_arc(to_vertex=neighbor, weight=weight)

        path.remove(to_vertex)

        # To different ways to calculate weight of arc
        # depending on status of vertex (checked or not checked).
        if to_vertex in not_checked:
            weight = (to_vertex.value - from_vertex.value +
                      max([0, *map(lambda arc: arc.weight, to_vertex.arcs)]) + 1)
        else:
            weight = max([float("-inf"), *map(lambda arc: arc.weight, to_vertex.arcs)]) - 1

        logger.debug(f"Created arc\n"
                     f"\tfrom {from_vertex}\n"
                     f"\tto {to_vertex}\n"
                     f"\tweight {weight}")

        return weight

    # Select vertexes to which we can move and calculate arcs to them.
    for neighbor in start_vertex.adjacent:
        if neighbor.value >= start_vertex.value:
            weight = calculate_arc_weight(start_vertex, neighbor)
            start_vertex.add_arc(to_vertex=neighbor, weight=weight)


def recalculate_values(graph: Graph, start_vertex: Vertex, not_checked: Set[Vertex]) -> None:
    """Recalculate values of vertexes depending on whether the vertex is checked."""

    # Check given attributes.
    if start_vertex not in graph:
        raise ValueError("Given vertex not in graph.")

    # Create temp graph to calculate new values depends on the distance.
    temp_graph = graph.deepcopy(with_arcs=False, with_values=False)
    fill_graph_values(graph=temp_graph,
                      start_vertex=temp_graph[start_vertex.id_])

    # Realcute values based on whether the vertex is checked.
    for vertex in graph:
        if vertex in not_checked:
            vertex.value += temp_graph[vertex.id_].value
        else:
            vertex.value = temp_graph[vertex.id_].value


def walk_graph(drone: Drone) -> None:
    """
    Walk prioritized through all vertexes in drone's graph.
    Drone always want to move to not checked vertex with maximum value.
    """

    not_checked = set(drone.graph)
    not_checked.remove(drone.position)

    # Walk in graph until all vertexes is checked.
    while not_checked:
        calculate_arcs(graph=drone.graph,
                       start_vertex=drone.position,
                       not_checked=not_checked)

        # Perform a gradient descent to a local maximum.
        while drone.position.arcs:
            new_position = max(drone.position.arcs, key=lambda arc: arc.weight).to_vertex
            if new_position in not_checked:
                not_checked.remove(new_position)

            drone.move_to(new_position)

            if drone.someone_found_object:
                return

            found_object, path_to_image = drone.recognise()

            if found_object:
                drone.send_image(path_to_image)

                return

        recalculate_values(graph=drone.graph,
                           start_vertex=drone.position,
                           not_checked=not_checked)


def return_to_base(drone: Drone) -> None:
    """Return to base from current position, going through selected vertexes by drone."""

    def find_path(graph: Graph, from_vertex: Vertex, to_vertex: Vertex) -> List[Vertex]:
        """Find shortest path using Dijkstra’s algorithm."""

        if from_vertex not in graph or to_vertex not in graph:
            raise ValueError("Given vertex not in graph.")

        not_checked = set(graph)
        distances = {vertex: float("inf") for vertex in graph}  # distances between to_vertex and vertex.
        ancestor = {}

        distances[from_vertex] = 0

        # Check every vertex and set the distances to neighbors and ancestors in path.
        while not_checked:
            vertex = min(not_checked, key=lambda vertex: distances[vertex])

            logger.debug(f"find_path: {vertex.adjacent=}")
            logger.debug(f"find_path: {distances[vertex]=}")

            for neighbor in vertex.adjacent:
                current_distance = distances[neighbor]
                new_distance = distances[vertex] + distance(vertex, neighbor)

                if new_distance < current_distance:
                    distances[neighbor] = new_distance
                    ancestor[neighbor] = vertex

            not_checked.remove(vertex)

        path: List[Vertex] = []
        vertex = to_vertex

        logger.debug(f"find_path: {not_checked=}")
        logger.debug(f"find_path: {ancestor=}")
        logger.debug(f"find_path: {distances=}")

        while vertex != from_vertex:
            path.insert(0, vertex)
            vertex = ancestor[vertex]

        return path

    path = find_path(graph=drone.graph,
                     from_vertex=drone.position,
                     to_vertex=drone.graph[CONFIGURATION["drones initial positions"][str(drone.id_)]])

    for vertex in path:
        drone.move_to(vertex)


def main(drone_id: int) -> None:
    """Main function."""

    # Initialize graph.
    global_graph = initialize_graph_from_dict(FIELD, CONFIGURATION["drone flight altitude"])

    logger.info(f"Drone {drone_id} has completed initialization of the global graph.")
    logger.debug(f"Drone {drone_id} global graph: {global_graph}")

    drone = Drone(id_=drone_id,
                  position=global_graph[CONFIGURATION["drones initial positions"][str(drone_id)]],
                  global_graph=global_graph)

    logger.info(f"Drone {drone_id} initialized:\n"
                f"\t{drone}")

    drone.wait_start_command()

    logger.info(f"Drone {drone_id} has started initialization of values in global graph.")

    fill_global_values(drone)

    logger.info(f"Drone {drone_id} has completed initialization of values in global graph.")
    logger.debug(f"Drone {drone_id} global graph: {global_graph}")

    logger.info(f"Drone {drone_id} has started splitting of global graph.")

    split_graph(drone)

    logger.info(f"Drone {drone_id} has completed splitting of global graph.")
    logger.debug(f"Drone {drone_id} graph: {drone.graph}")

    logger.info(f"Drone {drone_id} has started walk in his graph.")

    drone.takeoff()
    walk_graph(drone)

    logger.info(f"Drone {drone_id} has completed his work.")
    logger.info(f"Drone {drone_id} returning to base.")

    return_to_base(drone)

    drone.landing()

    logger.debug(f"Drone {drone_id} has landed.")

    drone.reset_default_attributes()


if __name__ == "__main__":
    rospy.init_node("gs_swarm_drones_node")

    _configuration_path = rospy.get_param(rospy.search_param("configuration_path"))
    _field_path = rospy.get_param(rospy.search_param("field_path"))

    with open(_configuration_path) as configuration_file, \
            open(_field_path) as field_file:
        CONFIGURATION = json.load(configuration_file)
        FIELD = json.load(field_file)

    main(CONFIGURATION["drone id"])
