import logging
import math
import os
import time
import typing

from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import map_processing_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseWallet, LeaseKeepAlive, Lease
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.robot import Robot
from bosdyn.client.robot_state import RobotStateClient
from google.protobuf import wrappers_pb2


class SpotGraphNav:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        graph_nav_client: GraphNavClient,
        map_processing_client: MapProcessingServiceClient,
        robot_state_client: RobotStateClient,
        lease_client: LeaseClient,
    ):
        self._robot = robot
        self._logger = logger
        self._graph_nav_client = graph_nav_client
        self._map_processing_client = map_processing_client
        self._robot_state_client = robot_state_client
        self._lease_client = lease_client
        self._lease_wallet: LeaseWallet = self._lease_client.lease_wallet

        self._init_current_graph_nav_state()

    def _get_lease(self) -> Lease:
        self._lease = self._lease_wallet.get_lease()
        return self._lease

    def _init_current_graph_nav_state(self):
        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = {}  # maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = {}  # maps id to waypoint snapshot
        self._current_edge_snapshots = {}  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = {}
        self._current_anchored_world_objects = (
            {}
        )  # maps object id to a (wo, waypoint, fiducial)
        self._current_anchors = {}  # maps anchor id to anchor

    def list_graph(self) -> typing.List[str]:
        """List waypoint ids of graph_nav
        Args:
          upload_path : Path to the root directory of the map.
        """
        ids, eds = self._list_graph_waypoint_and_edge_ids()

        return [
            v
            for k, v in sorted(
                ids.items(), key=lambda id: int(id[0].replace("waypoint_", ""))
            )
        ]

    def navigate_initial_localization(
        self,
        upload_path: str,
        initial_localization_fiducial: bool = True,
        initial_localization_waypoint: typing.Optional[str] = None,
    ):
        """Navigate with graphnav.

        Args:
           upload_path : Path to the root directory of the map.
           navigate_to : Waypont id string for where to goal
           initial_localization_fiducial : Tells the initializer whether to use fiducials
           initial_localization_waypoint : Waypoint id string of current robot position (optional)
        """
        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path and upload_path[-1] == "/":
            upload_filepath = upload_path[:-1]
        else:
            upload_filepath = upload_path

        # Boolean indicating the robot's power state.
        power_state = self._robot_state_client.get_robot_state().power_state
        self._started_powered_on = power_state.motor_power_state == power_state.STATE_ON
        self._powered_on = self._started_powered_on

        # Claim lease, power on robot, start graphnav.
        self._lease = self._get_lease()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        if upload_filepath:
            self.clear_graph()
            self._upload_graph_and_snapshots(upload_filepath)
        else:
            self._download_current_graph()
            self._logger.info(
                "Re-using existing graph on robot. Check that the correct graph is loaded!"
            )
        if initial_localization_fiducial:
            self.set_initial_localization_fiducial()
        if initial_localization_waypoint:
            self.set_initial_localization_waypoint([initial_localization_waypoint])
        self._list_graph_waypoint_and_edge_ids()
        self._get_localization_state()

        return True, "Localization done!"

    def navigate_to_existing_waypoint(self, waypoint_id: str):
        """Navigate to an existing waypoint.
        Args:
            waypoint_id : Waypoint id string for where to go
        """
        self._get_localization_state()
        resp = self._navigate_to(waypoint_id)
        return resp

    def navigate_through_route(self, waypoint_ids: typing.List[str]):
        """
        Args:
            waypoint_ids: List[str] of waypoints to follow
        """
        self._get_localization_state()
        self._logger.info(f"Waypoint ids: {','.join(waypoint_ids)}")
        resp = self._navigate_route(waypoint_ids)
        return resp

    def download_navigation_graph(self, download_path: str) -> typing.List[str]:
        """Download the navigation graph.
        Args:
            download_path : Path to the root directory of the map.
        """
        self._download_filepath = download_path
        self._download_full_graph()
        return self.list_graph()

    def navigation_close_loops(
        self, close_fiducial_loops: bool, close_odometry_loops: bool
    ) -> typing.Tuple[bool, str]:
        return self._auto_close_loops(close_fiducial_loops, close_odometry_loops)

    def optmize_anchoring(self) -> typing.Tuple[bool, str]:
        return self._optimize_anchoring()

    def _write_bytes_while_download(self, filepath: str, data: bytes):
        """Write data to a file.
        Args:
            filepath (str) : Path of file where data will be written.
            data (bytes) : Bytes of data"""
        directory = os.path.dirname(filepath)
        os.makedirs(directory, exist_ok=True)
        with open(filepath, "wb+") as f:
            f.write(data)
            f.close()

    def _download_graph_and_snapshots(
        self, download_path: str
    ) -> typing.Tuple[bool, str]:
        """Download the graph and snapshots from the robot.
        Args:
            download_path (str): Directory where graph and snapshots are downloaded from robot.
        Returns:
            success (bool): success flag
            message (str): message"""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            return False, "Failed to download the graph."
        graph_bytes = graph.SerializeToString()
        self._write_bytes_while_download(
            os.path.join(download_path, "graph"), graph_bytes
        )
        # Download the waypoint and edge snapshots.
        for waypoint in graph.waypoints:
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id
                )
            except Exception:
                self._logger.warning(
                    "Failed to download waypoint snapshot: %s", waypoint.snapshot_id
                )
                continue
            self._write_bytes_while_download(
                os.path.join(download_path, "waypoint_snapshots", waypoint.snapshot_id),
                waypoint_snapshot.SerializeToString(),
            )
        for edge in graph.edges:
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(
                    edge.snapshot_id
                )
            except Exception:
                self._logger.warning(
                    "Failed to download edge snapshot: %s", edge.snapshot_id
                )
                continue
            self._write_bytes_while_download(
                os.path.join(download_path, "edge_snapshots", edge.snapshot_id),
                edge_snapshot.SerializeToString(),
            )
        return True, "Success"

    def clear_graph(self) -> typing.Tuple[bool, str]:
        """Clear the state of the map on the robot, removing all waypoints and edges in the RAM of the robot.
        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        try:
            self._clear_graph()
            return True, "Success"
        except Exception as e:
            return (
                False,
                f"Got an error while clearing a graph and snanshots in a robot: {e}",
            )

    def upload_graph(self, upload_path: str) -> typing.Tuple[bool, str]:
        """Upload the specified graph and snapshots from local to a robot.
        While this method, if there are snapshots already in the robot, they will be loaded from the robot's disk without uploading.
        Graph and snapshots to be uploaded should be placed like
        Directory specified with upload_path arg
          |
          +-- graph
          |
          +-- waypoint_snapshots/
          |     |
          |     +-- waypont snapshot files
          |
          +-- edge_snapshots/
                |
                +-- edge snapshot files
        Args:
            upload_path (str): Path to the directory of the map.
        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        try:
            self._upload_graph_and_snapshots(upload_path)
            return True, "Success"
        except Exception as e:
            return (
                False,
                f"Got an error while uploading a graph and snapshots to a robot: {e}",
            )

    def download_graph(self, download_path: str) -> typing.Tuple[bool, str]:
        """Download current graph and snapshots in the robot to the specified directory.
        Args:
            download_path (str): Directory where graph and snapshots are downloaded from robot.
        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        try:
            success, message = self._download_graph_and_snapshots(
                download_path=download_path
            )
            return success, message
        except Exception as e:
            return (
                False,
                f"Got an error during downloading graph and snapshots from the robot: {e}",
            )

    ## Copied from https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_command_line/recording_command_line.py and https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/graph_nav_command_line/graph_nav_command_line.py with minor modifications
    # Copyright (c) 2020 Boston Dynamics, Inc.  All rights reserved.
    #
    # Downloading, reproducing, distributing or otherwise using the SDK Software
    # is subject to the terms and conditions of the Boston Dynamics Software
    # Development Kit License (20191101-BDSDK-SL).
    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        self._logger.info(f"Got localization: \n{str(state.localization)}")
        odom_tform_body = get_odom_tform_body(
            state.robot_kinematics.transforms_snapshot
        )
        self._logger.info(
            f"Got robot state in kinematic odometry frame: \n{str(odom_tform_body)}"
        )

    def set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            ko_tform_body=current_odom_tform_body,
        )

    def set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            self._logger.error("No waypoint specified to initialize to.")
            return
        destination_waypoint = self._find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            self._logger.error("Failed to find waypoint id.")
            return

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot
        ).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance=0.2,
            max_yaw=20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body,
        )

    def _download_current_graph(self):
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Empty graph.")
            return
        self._current_graph = graph
        return graph

    def _download_full_graph(self, *args):
        """Download the graph and snapshots from the robot."""
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.info("Failed to download the graph.")
            return
        self._write_full_graph(graph)
        self._logger.info(
            "Graph downloaded with {} waypoints and {} edges".format(
                len(graph.waypoints), len(graph.edges)
            )
        )
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints)
        self._download_and_write_edge_snapshots(graph.edges)

    def _write_full_graph(self, graph):
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        self._write_bytes(self._download_filepath, "/graph", graph_bytes)

    def _download_and_write_waypoint_snapshots(self, waypoints):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            if len(waypoint.snapshot_id) == 0:
                continue
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id
                )
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self._logger.error(
                    "Failed to download waypoint snapshot: " + waypoint.snapshot_id
                )
                continue
            self._write_bytes(
                self._download_filepath + "/waypoint_snapshots",
                "/" + waypoint.snapshot_id,
                waypoint_snapshot.SerializeToString(),
            )
            num_waypoint_snapshots_downloaded += 1
            self._logger.info(
                "Downloaded {} of the total {} waypoint snapshots.".format(
                    num_waypoint_snapshots_downloaded, len(waypoints)
                )
            )

    def _download_and_write_edge_snapshots(self, edges):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        num_to_download = 0
        for edge in edges:
            if len(edge.snapshot_id) == 0:
                continue
            num_to_download += 1
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(
                    edge.snapshot_id
                )
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self._logger.error(
                    "Failed to download edge snapshot: " + edge.snapshot_id
                )
                continue
            self._write_bytes(
                self._download_filepath + "/edge_snapshots",
                "/" + edge.snapshot_id,
                edge_snapshot.SerializeToString(),
            )
            num_edge_snapshots_downloaded += 1
            self._logger.info(
                "Downloaded {} of the total {} edge snapshots.".format(
                    num_edge_snapshots_downloaded, num_to_download
                )
            )

    def _write_bytes(self, filepath: str, filename: str, data):
        """Write data to a file."""
        os.makedirs(filepath, exist_ok=True)
        with open(filepath + filename, "wb+") as f:
            f.write(data)
            f.close()

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._download_current_graph()

        localization_id = (
            self._graph_nav_client.get_localization_state().localization.waypoint_id
        )

        # Update and print waypoints and edges
        (
            self._current_annotation_name_to_wp_id,
            self._current_edges,
        ) = self._update_waypoints_and_edges(graph, localization_id, self._logger)
        return self._current_annotation_name_to_wp_id, self._current_edges

    def _upload_graph_and_snapshots(self, upload_filepath: str):
        """Upload the graph and snapshots to the robot."""
        self._lease = self._get_lease()
        self._logger.info("Loading the graph from disk into local storage...")
        with open(os.path.join(upload_filepath, "graph"), "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            self._logger.info(
                "Loaded graph has {} waypoints and {} edges".format(
                    len(self._current_graph.waypoints), len(self._current_graph.edges)
                )
            )
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            if len(waypoint.snapshot_id) == 0:
                continue
            waypoint_filepath = os.path.join(
                upload_filepath, "waypoint_snapshots", waypoint.snapshot_id
            )
            if not os.path.exists(waypoint_filepath):
                continue
            with open(waypoint_filepath, "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[
                    waypoint_snapshot.id
                ] = waypoint_snapshot

                for fiducial in waypoint_snapshot.objects:
                    if not fiducial.HasField("apriltag_properties"):
                        continue

                    str_id = str(fiducial.apriltag_properties.tag_id)
                    if (
                        str_id in self._current_anchored_world_objects
                        and len(self._current_anchored_world_objects[str_id]) == 1
                    ):
                        # Replace the placeholder tuple with a tuple of (wo, waypoint, fiducial).
                        anchored_wo = self._current_anchored_world_objects[str_id][0]
                        self._current_anchored_world_objects[str_id] = (
                            anchored_wo,
                            waypoint,
                            fiducial,
                        )
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            if len(edge.snapshot_id) == 0:
                continue
            edge_filepath = os.path.join(
                upload_filepath, "edge_snapshots", edge.snapshot_id
            )
            if not os.path.exists(edge_filepath):
                continue
            with open(edge_filepath, "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
        for anchor in self._current_graph.anchoring.anchors:
            self._current_anchors[anchor.id] = anchor
        # Upload the graph to the robot.
        self._logger.info("Uploading the graph and snapshots to the robot...")
        if self._lease is None:
            self._logger.error(
                "Graph nav module did not have a lease to the robot. Claim it before attempting to upload the graph "
                "and snapshots."
            )
            return

        self._graph_nav_client.upload_graph(
            lease=self._lease.lease_proto, graph=self._current_graph
        )
        # Upload the snapshots to the robot.
        for waypoint_snapshot in self._current_waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self._logger.info("Uploaded {}".format(waypoint_snapshot.id))
        for edge_snapshot in self._current_edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self._logger.info("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            self._logger.info(
                "Upload complete! The robot is currently not localized to the map; please localize the robot using a "
                "fiducial before attempting a navigation command."
            )

    def _navigate_to(self, waypoint_id: str) -> typing.Tuple[bool, str]:
        """Navigate to a specific waypoint."""
        self._lease = self._get_lease()
        destination_waypoint = self._find_unique_waypoint_id(
            waypoint_id,
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            self._logger.error(
                "Failed to find the appropriate unique waypoint id for the navigation command."
            )
            return (
                False,
                "Failed to find the appropriate unique waypoint id for the navigation command.",
            )

        # Stop the lease keepalive and create a new sublease for graphnav.
        self._lease = self._lease_wallet.advance()
        sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate to the destination waypoint.
        is_finished = False
        nav_to_cmd_id = -1
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_to_cmd_id = self._graph_nav_client.navigate_to(
                destination_waypoint, 1.0, leases=[sublease.lease_proto]
            )
            time.sleep(0.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete.
            is_finished = self._check_success(nav_to_cmd_id)

        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        status = self._graph_nav_client.navigation_feedback(nav_to_cmd_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            return True, "Successfully completed the navigation commands!"
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            return (
                False,
                "Robot got lost when navigating the route, the robot will now sit down.",
            )
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            return (
                False,
                "Robot got stuck when navigating the route, the robot will now sit down.",
            )
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            return False, "Robot is impaired."
        else:
            return False, "Navigation command is not complete yet."

    def _navigate_route(
        self, waypoint_ids: typing.List[str]
    ) -> typing.Tuple[bool, str]:
        """Navigate through a specific route of waypoints.
        Note that each waypoint must have an edge between them, aka be adjacent.
        """
        for i in range(len(waypoint_ids)):
            waypoint_ids[i] = self._find_unique_waypoint_id(
                waypoint_ids[i],
                self._current_graph,
                self._current_annotation_name_to_wp_id,
                self._logger,
            )
            if not waypoint_ids[i]:
                self._logger.error(
                    "navigate_route: Failed to find the unique waypoint id."
                )
                return False, "Failed to find the unique waypoint id."

        edge_ids_list = []
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                self._logger.error(
                    f"Failed to find an edge between waypoints: {start_wp} and {end_wp}"
                )
                return (
                    False,
                    f"Failed to find an edge between waypoints: {start_wp} and {end_wp}",
                )

        # Stop the lease keepalive and create a new sublease for graphnav.
        self._lease = self._lease_wallet.advance()
        sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate a specific route.
        route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)
        is_finished = False
        while not is_finished:
            # Issue the route command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_route_command_id = self._graph_nav_client.navigate_route(
                route, cmd_duration=1.0, leases=[sublease.lease_proto]
            )
            time.sleep(0.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the route is complete.
            is_finished = self._check_success(nav_route_command_id)

            self._lease = self._lease_wallet.advance()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        return True, "Finished navigating route!"

    def _clear_graph(self, *args) -> bool:
        """Clear the state of the map on the robot, removing all waypoints and edges in the RAM of the robot"""
        self._lease = self._get_lease()
        result = self._graph_nav_client.clear_graph(lease=self._lease.lease_proto)
        self._init_current_graph_nav_state()
        return result

    def _check_success(self, command_id: int = -1) -> bool:
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL
        ):
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self._logger.error(
                "Robot got lost when navigating the route, the robot will now sit down."
            )
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self._logger.error(
                "Robot got stuck when navigating the route, the robot will now sit down."
            )
            return True
        elif (
            status.status
            == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED
        ):
            self._logger.error("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _match_edge(
        self,
        current_edges: typing.Dict[str, typing.List[str]],
        waypoint1: str,
        waypoint2: str,
    ) -> typing.Optional[map_pb2.Edge.Id]:
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint2, to_waypoint=waypoint1
                    )
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(
                        from_waypoint=waypoint1, to_waypoint=waypoint2
                    )
        return None

    def _auto_close_loops(
        self, close_fiducial_loops: bool, close_odometry_loops: bool, *args
    ):
        """Automatically find and close all loops in the graph."""
        response: map_processing_pb2.ProcessTopologyResponse = (
            self._map_processing_client.process_topology(
                params=map_processing_pb2.ProcessTopologyRequest.Params(
                    do_fiducial_loop_closure=wrappers_pb2.BoolValue(
                        value=close_fiducial_loops
                    ),
                    do_odometry_loop_closure=wrappers_pb2.BoolValue(
                        value=close_odometry_loops
                    ),
                ),
                modify_map_on_server=True,
            )
        )
        self._logger.info(
            "Created {} new edge(s).".format(len(response.new_subgraph.edges))
        )
        if response.status == map_processing_pb2.ProcessTopologyResponse.STATUS_OK:
            return True, "Successfully closed loops."
        elif (
            response.status
            == map_processing_pb2.ProcessTopologyResponse.STATUS_MISSING_WAYPOINT_SNAPSHOTS
        ):
            return False, "Missing waypoint snapshots."
        elif (
            response.status
            == map_processing_pb2.ProcessTopologyResponse.STATUS_INVALID_GRAPH
        ):
            return False, "Invalid graph."
        elif (
            response.status
            == map_processing_pb2.ProcessTopologyResponse.STATUS_MAP_MODIFIED_DURING_PROCESSING
        ):
            return False, "Map modified during processing."
        else:
            return False, "Unknown error during map processing."

    def _optimize_anchoring(self, *args):
        """Call anchoring optimization on the server, producing a globally optimal reference frame for waypoints to be expressed in."""
        response: map_processing_pb2.ProcessAnchoringResponse = (
            self._map_processing_client.process_anchoring(
                params=map_processing_pb2.ProcessAnchoringRequest.Params(),
                modify_anchoring_on_server=True,
                stream_intermediate_results=False,
            )
        )
        if response.status == map_processing_pb2.ProcessAnchoringResponse.STATUS_OK:
            self._logger.info(
                "Optimized anchoring after {} iteration(s).".format(response.iteration)
            )
            return True, "Successfully optimized anchoring."
        else:
            self._logger.error("Error optimizing {}".format(response))
            status_mapping = {
                map_processing_pb2.ProcessAnchoringResponse.STATUS_MISSING_WAYPOINT_SNAPSHOTS: (
                    "Missing waypoint snapshots."
                ),
                map_processing_pb2.ProcessAnchoringResponse.STATUS_INVALID_GRAPH: "Invalid graph.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_OPTIMIZATION_FAILURE: "Optimization failure.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_INVALID_PARAMS: "Invalid parameters.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_CONSTRAINT_VIOLATION: "Constraint violation.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_MAX_ITERATIONS: "Max iterations, timeout.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_MAX_TIME: "Max time reached, timeout.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_INVALID_HINTS: "Invalid hints.",
                map_processing_pb2.ProcessAnchoringResponse.STATUS_MAP_MODIFIED_DURING_PROCESSING: (
                    "Map modified during processing."
                ),
                map_processing_pb2.ProcessAnchoringResponse.STATUS_INVALID_GRAVITY_ALIGNMENT: (
                    "Invalid gravity alignment."
                ),
            }
            if response.status in status_mapping:
                return False, status_mapping[response.status]
            return False, "Unknown error during anchoring optimization."

    def _id_to_short_code(self, id: str):
        """Convert a unique id to a 2 letter short code."""
        tokens = id.split("-")
        if len(tokens) > 2:
            return f"{tokens[0][0]}{tokens[1][0]}"
        return None

    def _pretty_print_waypoints(
        self,
        waypoint_id: str,
        waypoint_name: str,
        short_code_to_count: typing.Dict[str, int],
        localization_id: str,
        logger: logging.Logger,
    ):
        short_code = self._id_to_short_code(waypoint_id)
        if short_code is None or short_code_to_count[short_code] != 1:
            # If the short code is not valid/unique, don't show it.
            short_code = "  "

        logger.info(
            "%s Waypoint name: %s id: %s short code: %s"
            % (
                "->" if localization_id == waypoint_id else "  ",
                waypoint_name,
                waypoint_id,
                short_code,
            )
        )

    def _find_unique_waypoint_id(
        self,
        short_code: str,
        graph: map_pb2.Graph,
        name_to_id: typing.Dict[str, str],
        logger: logging.Logger,
    ):
        """Convert either a 2 letter short code or an annotation name into the associated unique id."""
        if len(short_code) != 2:
            # Not a short code, check if it is an annotation name (instead of the waypoint id).
            if short_code in name_to_id:
                # Short code is an waypoint's annotation name. Check if it is paired with a unique waypoint id.
                if name_to_id[short_code] is not None:
                    # Has an associated waypoint id!
                    return name_to_id[short_code]
                else:
                    logger.error(
                        "The waypoint name %s is used for multiple different unique waypoints. Please use"
                        + "the waypoint id." % (short_code)
                    )
                    return None
            # Also not an waypoint annotation name, so we will operate under the assumption that it is a
            # unique waypoint id.
            return short_code

        ret = short_code
        for waypoint in graph.waypoints:
            if short_code == self._id_to_short_code(waypoint.id):
                if ret != short_code:
                    return short_code  # Multiple waypoints with same short code.
                ret = waypoint.id
        return ret

    def _update_waypoints_and_edges(
        self, graph: map_pb2.Graph, localization_id: str, logger: logging.Logger
    ) -> typing.Tuple[typing.Dict[str, str], typing.Dict[str, str]]:
        """Update and print waypoint ids and edge ids."""
        name_to_id = dict()
        edges = dict()

        short_code_to_count = {}
        waypoint_to_timestamp = []
        for waypoint in graph.waypoints:
            # Determine the timestamp that this waypoint was created at.
            timestamp = -1.0
            try:
                timestamp = (
                    waypoint.annotations.creation_time.seconds
                    + waypoint.annotations.creation_time.nanos / 1e9
                )
            except:
                # Must be operating on an older graph nav map, since the creation_time is not
                # available within the waypoint annotations message.
                pass
            waypoint_to_timestamp.append(
                (waypoint.id, timestamp, waypoint.annotations.name)
            )

            # Determine how many waypoints have the same short code.
            short_code = self._id_to_short_code(waypoint.id)
            if short_code not in short_code_to_count:
                short_code_to_count[short_code] = 0
            short_code_to_count[short_code] += 1

            # Add the annotation name/id into the current dictionary.
            waypoint_name = waypoint.annotations.name
            if waypoint_name:
                if waypoint_name in name_to_id:
                    # Waypoint name is used for multiple different waypoints, so set the waypoint id
                    # in this dictionary to None to avoid confusion between two different waypoints.
                    name_to_id[waypoint_name] = None
                else:
                    # First time we have seen this waypoint annotation name. Add it into the dictionary
                    # with the respective waypoint unique id.
                    name_to_id[waypoint_name] = waypoint.id

        # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
        # fallback to sorting by annotation name.
        waypoint_to_timestamp = sorted(
            waypoint_to_timestamp, key=lambda x: (x[1], x[2])
        )

        # Print out the waypoints name, id, and short code in a ordered sorted by the timestamp from
        # when the waypoint was created.
        logger.info("%d waypoints:" % len(graph.waypoints))
        for waypoint in waypoint_to_timestamp:
            self._pretty_print_waypoints(
                waypoint[0], waypoint[2], short_code_to_count, localization_id, logger
            )

        for edge in graph.edges:
            if edge.id.to_waypoint in edges:
                if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                    edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
            else:
                edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
            logger.info(
                f"(Edge) from waypoint id: {edge.id.from_waypoint} and to waypoint id: {edge.id.to_waypoint}"
            )

        return name_to_id, edges
