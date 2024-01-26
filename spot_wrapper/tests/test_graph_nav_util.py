#!/usr/bin/env python3
import logging

from bosdyn.api.graph_nav import map_pb2

from spot_wrapper.spot_graph_nav import SpotGraphNav


class MockSpotGraphNav(SpotGraphNav):
    def __init__(self) -> None:
        pass


# Create a mock SpotGraphNav object to access utility methods
graph_nav_util = MockSpotGraphNav()


class TestGraphNavUtilShortCode:
    def test_id_to_short_code(self):
        assert graph_nav_util._id_to_short_code("ebony-pug-mUzxLNq.TkGlVIxga+UKAQ==") == "ep"
        assert graph_nav_util._id_to_short_code("erose-simian-sug9xpxhCxgft7Mtbhr98A==") == "es"


class TestGraphNavUtilFindUniqueWaypointId:
    def test_short_code(self):
        # Set up
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)
        self.graph = map_pb2.Graph()
        self.name_to_id = {"ABCDE": "Node1"}
        # Test normal short code
        assert graph_nav_util._find_unique_waypoint_id("AC", self.graph, self.name_to_id, self.logger) == "AC"
        # Test annotation name that is known
        assert graph_nav_util._find_unique_waypoint_id("ABCDE", self.graph, self.name_to_id, self.logger) == "Node1"
        # Test annotation name that is unknown
        assert graph_nav_util._find_unique_waypoint_id("ABCDEF", self.graph, self.name_to_id, self.logger) == "ABCDEF"

    def test_short_code_with_graph(self):
        # Set up
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)
        self.graph = map_pb2.Graph()
        self.name_to_id = {"ABCDE": "Node1"}

        # Test short code that is in graph
        self.graph.waypoints.add(id="AB-CD-EF")
        assert graph_nav_util._find_unique_waypoint_id("AC", self.graph, self.name_to_id, self.logger) == "AB-CD-EF"
        # Test short code that is not in graph
        assert graph_nav_util._find_unique_waypoint_id("AD", self.graph, self.name_to_id, self.logger) == "AD"
        # Test multiple waypoints with the same short code
        self.graph.waypoints.add(id="AB-CD-EF-1")
        assert graph_nav_util._find_unique_waypoint_id("AC", self.graph, self.name_to_id, self.logger) == "AC"


class TestGraphNavUtilUpdateWaypointsEdges:
    def test_empty_graph(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)

        # Test empty graph
        self.graph = map_pb2.Graph()
        self.localization_id = ""
        graph_nav_util._update_waypoints_and_edges(self.graph, self.localization_id, self.logger)
        assert len(self.graph.waypoints) == 0
        assert len(self.graph.edges) == 0

    def test_one_waypoint(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)

        # Test graph with 1 waypoint
        self.localization_id = ""
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(id=new_waypoint.id, annotations=new_waypoint.annotations)
        self.name_to_id, self.edges = graph_nav_util._update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        assert len(self.graph.waypoints) == 1
        assert len(self.graph.edges) == 0
        assert len(self.edges) == 0
        assert len(self.name_to_id) == 1
        assert self.name_to_id["Node1"] == "ABCDE"

    def test_two_waypoints_with_edge(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)

        # Test graph with 2 waypoints and an edge between them
        self.localization_id = ""
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(id=new_waypoint.id, annotations=new_waypoint.annotations)
        new_waypoint.id = "DE"
        new_waypoint.annotations.name = "Node2"
        self.graph.waypoints.add(id=new_waypoint.id, annotations=new_waypoint.annotations)
        new_edge = map_pb2.Edge.Id(from_waypoint="ABCDE", to_waypoint="DE")

        self.graph.edges.add(id=new_edge)
        self.name_to_id, self.edges = graph_nav_util._update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        assert len(self.graph.waypoints) == 2
        assert len(self.graph.edges) == 1
        assert len(self.edges) == 1
        assert self.edges["DE"][0] == "ABCDE"
        assert len(self.name_to_id) == 2
        assert self.name_to_id["Node1"] == "ABCDE"
        assert self.name_to_id["Node2"] == "DE"

    def test_two_waypoints_with_edge_and_localization(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)

        # Test graph with 2 waypoints and an edge between them. Mainly affects the pretty print.
        self.localization_id = "ABCDE"
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(id=new_waypoint.id, annotations=new_waypoint.annotations)
        new_waypoint.id = "DE"
        new_waypoint.annotations.name = "Node2"
        self.graph.waypoints.add(id=new_waypoint.id, annotations=new_waypoint.annotations)
        new_edge = map_pb2.Edge.Id(from_waypoint="ABCDE", to_waypoint="DE")

        self.graph.edges.add(id=new_edge)
        self.name_to_id, self.edges = graph_nav_util._update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        assert len(self.graph.waypoints) == 2
        assert len(self.graph.edges) == 1
        assert len(self.edges) == 1
        assert self.edges["DE"][0] == "ABCDE"
        assert len(self.name_to_id) == 2
        assert self.name_to_id["Node1"] == "ABCDE"
        assert self.name_to_id["Node2"] == "DE"
