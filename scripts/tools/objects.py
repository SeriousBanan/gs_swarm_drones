"""
Module with objects definitions.
"""

from dataclasses import dataclass, field
import json
from typing import Dict, Any, Set, Iterator, Tuple, Union
from time import sleep

from gs_board import BoardManager as _BoardManager  # pylint: disable=no-name-in-module
from gs_flight import (FlightController as _FlightController,  # pylint: disable=no-name-in-module
                       CallbackEvent as _CallbackEvent)  # pylint: disable=no-name-in-module
from gs_sensors import SensorManager as _SensorManager  # pylint: disable=no-name-in-module
import rospy  # pylint: disable=no-name-in-module

# pylint: disable=import-error, no-name-in-module
from gs_swarm_drones.srv import Recognise  # type: ignore
from gs_swarm_drones.srv import TransferData, TransferDataResponse  # type: ignore
# pylint: enable=import-error, no-name-in-module

from .setup_loggers import logger as _logger


__all__ = ["Coords", "Vertex", "Graph", "Drone"]


@dataclass(frozen=True)
class Coords:
    """Class of coords for different objects."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Arc:
    """Class of weighted arc to some vertex."""

    from_vertex: "Vertex"
    to_vertex: "Vertex"
    weight: float


@dataclass(unsafe_hash=True)
class Vertex:
    """Vertex class which contains it's id, coordinates, value and adjaced vertexes."""

    id_: int
    coords: Coords
    value: float = field(default=-1, hash=False)
    adjacent: Set["Vertex"] = field(default_factory=set, repr=False, hash=False)
    arcs: Set[Arc] = field(default_factory=set, repr=False, hash=False)

    def copy(self) -> "Vertex":
        """Return copy of the vertex without neighbors and arcs."""

        return Vertex(id_=self.id_,
                      coords=self.coords,
                      value=self.value)

    def add_neighbor(self, neighbor: "Vertex") -> None:
        """Create adjacent between current vertex and neighbor vertex."""

        self.adjacent.add(neighbor)
        neighbor.adjacent.add(self)

    def add_arc(self, to_vertex: "Vertex", weight: float = 0) -> None:
        """Create arc to vertex."""

        self.arcs.add(Arc(from_vertex=self,
                          to_vertex=to_vertex,
                          weight=weight))


@dataclass
class Graph:
    """
    Graph class.

    Support:
    - __setitem__
    - __getitem__
    - __iter__
    - __contains__ by vertex or vertex.id_.
    """

    vertexes: Dict[int, Vertex] = field(default_factory=dict)

    def add_vertex(self, vertex: Vertex) -> None:
        """Adding vertex to graph."""

        self.vertexes[vertex.id_] = vertex

    def deepcopy(self,
                 with_adjacent: bool = True,
                 with_arcs: bool = True,
                 with_values: bool = True) -> "Graph":
        """
        Create deepcopy of graph.

        If `with_adjacent` is False not creating edges between vertexes.
        If `with_arcs` is False not creating arcs between vertexes.
        If `with_values` is False setting vertex's values to None.
        """

        new_graph = Graph()

        for vertex in self:
            new_vertex = vertex.copy()
            if not with_values:
                new_vertex.value = -1

            new_graph.add_vertex(new_vertex)

        if not with_adjacent and not with_arcs:
            return new_graph

        for vertex in self:
            if with_adjacent:
                for neighbor in vertex.adjacent:
                    from_vertex = new_graph.vertexes[vertex.id_]
                    to_vertex = new_graph.vertexes[neighbor.id_]

                    from_vertex.add_neighbor(to_vertex)

            if with_arcs:
                for arc in vertex.arcs:
                    from_vertex = new_graph.vertexes[vertex.id_]
                    to_vertex = new_graph.vertexes[arc.to_vertex.id_]

                    from_vertex.add_arc(to_vertex=arc.to_vertex,
                                        weight=arc.weight)

        return new_graph

    def __setitem__(self, vertex_id: int, value: Vertex) -> None:
        self.vertexes[vertex_id] = value

    def __getitem__(self, vertex_id: int) -> Vertex:
        return self.vertexes[vertex_id]

    def __iter__(self) -> Iterator[Vertex]:
        return (vertex for vertex in self.vertexes.values())

    def __contains__(self, item: Union[int, Vertex]) -> bool:
        if isinstance(item, int):
            return item in self.vertexes

        return item in self.vertexes.values()


@dataclass
class Drone:
    """
    Drone class.

    Each drone contains:
    - id.
    - his position (In which Vertex).
    - link to global field.
    - personal graph.
    - his charge property.
    """

    id_: int
    position: Vertex
    global_graph: Graph
    graph: Graph = field(default_factory=Graph)
    found_object: bool = False
    someone_found_object: bool = False
    _board_manager: _BoardManager = field(default_factory=_BoardManager)
    _sensor_manager: _SensorManager = field(default_factory=_SensorManager)
    _flight_callback_event: int = 0
    _start: bool = False

    def __post_init__(self):
        # Need to  drone local position system.
        # Will shift the global position to the local coordinate system.
        self._initial_coords = self.position.coords

        # Wait until board starts.
        while not self._board_manager.runStatus():
            sleep(0.05)

        def flight_controller_callback(event_num):
            """Callback for flight controller."""

            self._flight_callback_event = event_num.data

        self._flight_controller = _FlightController(flight_controller_callback)

        rospy.wait_for_service("gs_swarm_drones/recognise")
        self._recognise_service = rospy.ServiceProxy("gs_swarm_drones/recognise", Recognise)

        rospy.wait_for_service("gs_swarm_drones/send_message")
        self._send_message_service = rospy.ServiceProxy("gs_swarm_drones/send_message", TransferData)

        rospy.wait_for_service("gs_swarm_drones/send_image")
        self._send_image_service = rospy.ServiceProxy("gs_swarm_drones/send_image", TransferData)

        def receive_message_callback(data):

            if data.data == "start":
                self._start = True
                return TransferDataResponse(True)

            try:
                message = json.loads(data.data)
                if message["found"]:
                    self.someone_found_object = True
            except (KeyError, json.decoder.JSONDecodeError):
                return TransferDataResponse(False)

            return TransferDataResponse(True)

        self._receive_message_service = rospy.Service("gs_swarm_drones/receive_message", TransferData, receive_message_callback)

    @property
    def charge(self) -> float:
        """Return charge of the Drone."""

        charge = self._sensor_manager.power()
        _logger.info(f"Drone {self.id_} charge: {charge}.")

        return self._sensor_manager.power()

    def _wait_flight_callback_event(self, event: int) -> None:
        while self._flight_callback_event != event:
            sleep(0.05)

        self._flight_callback_event = 0

    def wait_start_command(self):
        """Wait until operator send `start` command."""
        while not self._start:
            sleep(0.05)

    def takeoff(self) -> None:
        """Takeoff drone from the ground."""

        _logger.info(f"Drone {self.id_} starts engines.")

        # Send command to starts engines.
        responce_status = self._flight_controller.preflight()
        _logger.debug(f"Drone {self.id_} _flight_controller.preflight responce: {responce_status}.")

        # Wait until engines start.
        self._wait_flight_callback_event(_CallbackEvent.ENGINES_STARTED)

        _logger.info(f"Drone {self.id_} has started engines. Takeoff starts.")

        # Send command to takeoff.
        responce_status = self._flight_controller.takeoff()
        _logger.debug(f"Drone {self.id_} _flight_controller.takeoff responce: {responce_status}.")

        # Wait until takeoff complete.
        self._wait_flight_callback_event(_CallbackEvent.TAKEOFF_COMPLETE)

        sleep(3)

        self.move_to(self.position)

        _logger.info(f"Drone {self.id_} has completed takeoff.")

    def landing(self) -> None:
        """Land drone to the ground."""

        _logger.info(f"Drone {self.id_} starts lending.")

        # Send command to start landing
        responce_status = self._flight_controller.landing()
        _logger.debug(f"Drone {self.id_} _flight_controller.landing responce: {responce_status}")

        # Wait until landing complete.
        self._wait_flight_callback_event(_CallbackEvent.COPTER_LANDED)

        _logger.info(f"Drone {self.id_} has completed landing.")

        # Send command to stop engines.
        responce_status = self._flight_controller.disarm()
        _logger.debug(f"Drone {self.id_} _flight_controller.disarm responce: {responce_status}")

        _logger.info(f"Drone {self.id_} has stopped engines.")

    def move_to(self, vertex: Vertex) -> None:
        """Moving drone to vertex."""

        _logger.info(f"Drone {self.id_} starts moving to {vertex}.")

        # Send command to move to vertex.
        coords = vertex.coords
        responce_status = self._flight_controller.goToLocalPoint(coords.x - self._initial_coords.x,
                                                                 coords.y - self._initial_coords.y,
                                                                 coords.z)
        _logger.debug(f"Drone {self.id_} _flight_controller.goToLocalPoint responce: {responce_status}.")

        # Wait until drone reach point.
        self._wait_flight_callback_event(_CallbackEvent.POINT_REACHED)

        self.position = vertex
        _logger.info(f"Drone {self.id_} has reached {vertex}")

        self.send_message({
            "id": self.id_,
            "coords": {
                "x": self.position.coords.x,
                "y": self.position.coords.y,
                "z": self.position.coords.z,
            },
            "charge": self.charge,
            "found": self.found_object
        })

    def recognise(self) -> Tuple[bool, str]:
        """Recognise tank using camera."""

        _logger.info(f"Drone {self.id_} has started recognition.")

        response = self._recognise_service()

        _logger.debug(f"Drone {self.id_} gs_swarm_drones/recognise response: {response}")
        _logger.info(f"Drone {self.id_} has completed recognition.")

        self.found_object = response.found_object

        if self.found_object:
            _logger.info(f"Drone {self.id_} found object.")

        return response.found_object, response.result_path

    def send_message(self, data: dict) -> None:
        """Sending message to other drones."""

        _logger.info(f"Drone {self.id_} posting {data}.")
        response = self._send_message_service(json.dumps(data))

        _logger.debug(f"Drone {self.id_} gs_swarm_drones/send_message response: {response}")
        _logger.info(f"Drone {self.id_} has send image.")

    def send_image(self, image_path: str) -> None:
        """Sending image to operator."""

        _logger.info(f"Drone {self.id_} start sending image.")

        response = self._send_image_service(image_path)

        _logger.debug(f"Drone {self.id_} gs_swarm_drones/send_image response: {response}")
        _logger.info(f"Drone {self.id_} has send image.")


del dataclass, field
del Dict, Set, Iterator, Union, Any
