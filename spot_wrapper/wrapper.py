from dataclasses import dataclass
import functools
import logging
import math
import os
import time
import traceback
import threading
import typing

import bosdyn.client.auth
from bosdyn.api import arm_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api import lease_pb2
from bosdyn.api import point_cloud_pb2
from bosdyn.api import manipulation_api_pb2
from bosdyn.api import robot_command_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import robot_state_pb2
from bosdyn.api import synchronized_command_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api import world_object_pb2
from bosdyn.api import point_cloud_pb2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.client import robot_command
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import (
    EstopClient,
    EstopEndpoint,
    EstopKeepAlive,
)
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.payload_registration import PayloadNotAuthorizedError
from bosdyn.client.point_cloud import PointCloudClient, build_pc_request
from bosdyn.client.power import safe_power_off, PowerClient, power_on
from bosdyn.client.robot import UnregisteredServiceError, Robot
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.spot_check import SpotCheckClient
from bosdyn.client.time_sync import TimeSyncEndpoint
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.license import LicenseClient
from bosdyn.client import ResponseError, RpcError, create_standard_sdk

try:
    from bosdyn.choreography.client.choreography import (
        ChoreographyClient,
    )
    from .spot_dance import SpotDance

    HAVE_CHOREOGRAPHY = True
except ModuleNotFoundError:
    HAVE_CHOREOGRAPHY = False

from bosdyn.geometry import EulerZXY
from bosdyn.util import seconds_to_duration
from google.protobuf.duration_pb2 import Duration

SPOT_CLIENT_NAME = "ros_spot"
MAX_COMMAND_DURATION = 1e5
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"

### Release
from . import graph_nav_util

### Debug
# import graph_nav_util

from bosdyn.api import basic_command_pb2
from google.protobuf.timestamp_pb2 import Timestamp

from .spot_arm import SpotArm
from .spot_check import SpotCheck
from .spot_docking import SpotDocking
from .spot_eap import SpotEAP
from .spot_images import SpotImages
from .spot_world_objects import SpotWorldObjects

from .wrapper_helpers import RobotCommandData, RobotState

"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
point_cloud_sources = ["velodyne-point-cloud"]


def robotToLocalTime(timestamp: Timestamp, robot: Robot) -> Timestamp:
    """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

    Args:
        timestamp: google.protobuf.Timestamp
        robot: Robot handle to use to get the time skew
    Returns:
        google.protobuf.Timestamp
    """

    rtime = Timestamp()

    rtime.seconds = timestamp.seconds - robot.time_sync.endpoint.clock_skew.seconds
    rtime.nanos = timestamp.nanos - robot.time_sync.endpoint.clock_skew.nanos
    if rtime.nanos < 0:
        rtime.nanos = rtime.nanos + int(1e9)
        rtime.seconds = rtime.seconds - 1

    # Workaround for timestamps being incomplete
    if rtime.seconds < 0:
        rtime.seconds = 0
        rtime.nanos = 0

    return rtime


class MissingSpotArm(Exception):
    """Raised when the arm is not available on the robot"""

    def __init__(self, message="Spot arm not available"):
        # Call the base class constructor with the parameters it needs
        super().__init__(message)


class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__(
            "robot-state", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_state_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__(
            "robot-metrics", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_metrics_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncLease(AsyncPeriodicQuery):
    """Class to get lease state at regular intervals.  list_leases_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncLease, self).__init__(
            "lease", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_leases_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class AsyncIdle(AsyncPeriodicQuery):
    """
    Class to check if the robot is moving, and if not, command a stand with the set mobility parameters
    """

    def __init__(
        self,
        client: RobotCommandClient,
        logger: logging.Logger,
        rate: float,
        spot_wrapper,
    ) -> None:
        """
        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            spot_wrapper: A handle to the wrapper library
        """
        super(AsyncIdle, self).__init__("idle", client, logger, period_sec=1.0 / rate)
        self._spot_wrapper: SpotWrapper = spot_wrapper

    def _start_query(self) -> None:
        if self._spot_wrapper.last_stand_command is not None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper.last_stand_command
                )
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status
                )
                self._spot_wrapper.is_sitting = False
                if status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                    self._spot_wrapper.is_standing = True
                    self._spot_wrapper.last_stand_command = None
                elif (
                    status == basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS
                ):
                    self._spot_wrapper.is_standing = False
                else:
                    self._logger.warning("Stand command in unknown state")
                    self._spot_wrapper.is_standing = False
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._spot_wrapper.last_stand_command = None

        if self._spot_wrapper.last_sit_command is not None:
            try:
                self._spot_wrapper.is_standing = False
                response = self._client.robot_command_feedback(
                    self._spot_wrapper.last_sit_command
                )
                if (
                    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status
                    == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
                ):
                    self._spot_wrapper.is_sitting = True
                    self._spot_wrapper.last_sit_command = None
                else:
                    self._spot_wrapper.is_sitting = False
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._spot_wrapper.last_sit_command = None

        is_moving = False

        if self._spot_wrapper.last_velocity_command_time is not None:
            if time.time() < self._spot_wrapper.last_velocity_command_time:
                is_moving = True
            else:
                self._spot_wrapper.last_velocity_command_time = None

        if self._spot_wrapper.last_trajectory_command is not None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper.last_trajectory_command
                )
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
                )
                # STATUS_AT_GOAL always means that the robot reached the goal. If the trajectory command did not
                # request precise positioning, then STATUS_NEAR_GOAL also counts as reaching the goal
                if (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL
                    or (
                        status
                        == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL
                        and not self._spot_wrapper.last_trajectory_command_precise
                    )
                ):
                    self._spot_wrapper.at_goal = True
                    # Clear the command once at the goal
                    self._spot_wrapper.last_trajectory_command = None
                    self._spot_wrapper._trajectory_status_unknown = False
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL
                ):
                    is_moving = True
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL
                ):
                    is_moving = True
                    self._spot_wrapper.near_goal = True
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_UNKNOWN
                ):
                    self._spot_wrapper.trajectory_status_unknown = True
                    self._spot_wrapper.last_trajectory_command = None
                else:
                    self._logger.error(
                        "Received trajectory command status outside of expected range, value is {}".format(
                            status
                        )
                    )
                    self._spot_wrapper.last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._spot_wrapper.last_trajectory_command = None

        self._spot_wrapper.is_moving = is_moving

        # We must check if any command currently has a non-None value for its id. If we don't do this, this stand
        # command can cause other commands to be interrupted before they get to start
        if (
            self._spot_wrapper.is_standing
            and self._spot_wrapper._continually_try_stand
            and not self._spot_wrapper.is_moving
            and self._spot_wrapper.last_trajectory_command is not None
            and self._spot_wrapper.last_stand_command is not None
            and self._spot_wrapper.last_velocity_command_time is not None
            and self._spot_wrapper.last_docking_command is not None
        ):
            self._spot_wrapper.stand(False)


class AsyncEStopMonitor(AsyncPeriodicQuery):
    """Class to check if the estop endpoint is still valid

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        spot_wrapper: A handle to the wrapper library
    """

    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncEStopMonitor, self).__init__(
            "estop_alive", client, logger, period_sec=1.0 / rate
        )
        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if not self._spot_wrapper._estop_keepalive:
            self._logger.debug("No keepalive yet - the lease has not been claimed.")
            return

        last_estop_status = self._spot_wrapper._estop_keepalive.status_queue.queue[-1]
        if (
            last_estop_status[0]
            == self._spot_wrapper._estop_keepalive.KeepAliveStatus.ERROR
        ):
            self._logger.error(
                "Estop keepalive has an error: {}".format(last_estop_status[1])
            )
        elif (
            last_estop_status
            == self._spot_wrapper._estop_keepalive.KeepAliveStatus.DISABLED
        ):
            self._logger.error(
                "Estop keepalive is disabled: {}".format(last_estop_status[1])
            )
        else:
            # estop keepalive is ok
            pass


def try_claim(func=None, *, power_on=False):
    """
    Decorator which tries to acquire the lease before executing the wrapped function

    the _func=None and * args are required to allow this decorator to be used with or without arguments

    Args:
        func: Function that is being wrapped
        power_on: If true, power on after claiming the lease

    Returns:
        Decorator which will wrap the decorated function
    """
    # If this decorator is being used without the power_on arg, return it as if it was called with that arg specified
    if func is None:
        return functools.partial(try_claim, power_on=power_on)

    @functools.wraps(func)
    def wrapper_try_claim(self, *args, **kwargs):
        if self._get_lease_on_action:
            if power_on:
                # Power on is also wrapped by this decorator so if we request power on the lease will also be claimed
                response = self.power_on()
            else:
                response = self.claim()
            if not response[0]:
                return response
        return func(self, *args, **kwargs)

    return wrapper_try_claim


class SpotWrapper:
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""

    def __init__(
        self,
        username: str,
        password: str,
        hostname: str,
        robot_name: str,
        logger: logging.Logger,
        start_estop: bool = True,
        estop_timeout: float = 9.0,
        rates: typing.Optional[typing.Dict[str, float]] = None,
        callbacks: typing.Optional[typing.Dict[str, typing.Callable]] = None,
        use_take_lease: bool = False,
        get_lease_on_action: bool = False,
        continually_try_stand: bool = True,
        rgb_cameras: bool = True,
        payload_credentials_file: str = None,
    ) -> None:
        """
        Args:
            username: Username for authentication with the robot
            password: Password for authentication with the robot
            hostname: ip address or hostname of the robot
            robot_name: Optional name of the robot
            start_estop: If true, the wrapper will be an estop endpoint
            estop_timeout: Timeout for the estop in seconds. The SDK will check in with the wrapper at a rate of
                           estop_timeout/3 and if there is no communication the robot will execute a gentle stop.
            rates: Dictionary of rates to apply when retrieving various data from the robot # TODO this should be an object to be unambiguous
            callbacks: Dictionary of callbacks which should be called when certain data is retrieved # TODO this should be an object to be unambiguous
            use_take_lease: Use take instead of acquire to get leases. This will forcefully take the lease from any
                            other lease owner.
            get_lease_on_action: If true, attempt to acquire a lease when performing an action which requires a
                                 lease. Otherwise, the user must manually take the lease. This will also attempt to
                                 power on the robot for commands which require it - stand, rollover, self-right.
            continually_try_stand: If the robot expects to be standing and is not, command a stand.  This can result
                                   in strange behavior if you use the wrapper and tablet together.
            rgb_cameras: If the robot has only body-cameras with greyscale images, this must be set to false.
        """
        self._username = username
        self._password = password
        self._hostname = hostname
        self._payload_credentials_file = payload_credentials_file
        self._robot_name = robot_name
        self._rates = rates or {}
        self._callbacks = callbacks or {}
        self._use_take_lease = use_take_lease
        self._get_lease_on_action = get_lease_on_action
        self._continually_try_stand = continually_try_stand
        self._rgb_cameras = rgb_cameras
        self._frame_prefix = ""
        if robot_name is not None:
            self._frame_prefix = robot_name + "/"
        self._logger = logger
        self._estop_timeout = estop_timeout
        self._start_estop = start_estop
        self._keep_alive = True
        self._lease_keepalive = None
        self._valid = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._state = RobotState()
        self._trajectory_status_unknown = False
        self._command_data = RobotCommandData()

        self._point_cloud_requests = []
        for source in point_cloud_sources:
            self._point_cloud_requests.append(build_pc_request(source))

        try:
            self._sdk = create_standard_sdk(SPOT_CLIENT_NAME)
        except Exception as e:
            self._logger.error(f"Error creating SDK object: {e}")
            self._valid = False
            return
        if HAVE_CHOREOGRAPHY:
            self._sdk.register_service_client(ChoreographyClient)
        self._logger.info("Initialising robot at {}".format(self._hostname))
        self._robot = self._sdk.create_robot(self._hostname)

        authenticated = False
        if self._payload_credentials_file:
            authenticated = self.authenticate_from_payload_credentials(
                self._robot, self._payload_credentials_file, self._logger
            )
        else:
            authenticated = self.authenticate(
                self._robot, self._username, self._password, self._logger
            )

        if not authenticated:
            self._valid = False
            return

        if not self._robot:
            self._logger.error("Failed to create robot object")
            self._valid = False
            return

        self._logger.info("Creating clients...")
        initialised = False
        while not initialised:
            try:
                self._robot_state_client = self._robot.ensure_client(
                    RobotStateClient.default_service_name
                )
                self._world_objects_client = self._robot.ensure_client(
                    WorldObjectClient.default_service_name
                )
                self._robot_command_client = self._robot.ensure_client(
                    RobotCommandClient.default_service_name
                )
                self._graph_nav_client = self._robot.ensure_client(
                    GraphNavClient.default_service_name
                )
                self._power_client = self._robot.ensure_client(
                    PowerClient.default_service_name
                )
                self._lease_client = self._robot.ensure_client(
                    LeaseClient.default_service_name
                )
                self._lease_wallet = self._lease_client.lease_wallet
                self._image_client = self._robot.ensure_client(
                    ImageClient.default_service_name
                )
                self._estop_client = self._robot.ensure_client(
                    EstopClient.default_service_name
                )
                self._docking_client = self._robot.ensure_client(
                    DockingClient.default_service_name
                )
                self._spot_check_client = self._robot.ensure_client(
                    SpotCheckClient.default_service_name
                )
                self._license_client = self._robot.ensure_client(
                    LicenseClient.default_service_name
                )

                if HAVE_CHOREOGRAPHY:
                    if self._license_client.get_feature_enabled(
                        [ChoreographyClient.license_name]
                    )[ChoreographyClient.license_name]:
                        self._is_licensed_for_choreography = True
                        self._choreography_client = self._robot.ensure_client(
                            ChoreographyClient.default_service_name
                        )
                    else:
                        self._logger.info("Robot is not licensed for choreography")
                        self._is_licensed_for_choreography = False
                        self._choreography_client = None
                else:
                    self._logger.info("Choreography is not available.")
                    self._choreography_client = None
                    self._is_licensed_for_choreography = False

                try:
                    self._point_cloud_client = self._robot.ensure_client(
                        VELODYNE_SERVICE_NAME
                    )
                except UnregisteredServiceError:
                    self._point_cloud_client = None
                    self._logger.info("No velodyne point cloud service is available.")

                if self._robot.has_arm():
                    self._manipulation_api_client = self._robot.ensure_client(
                        ManipulationApiClient.default_service_name
                    )
                else:
                    self._manipulation_api_client = None
                    self._logger.info("Manipulation API is not available.")

                initialised = True
            except Exception as e:
                sleep_secs = 15
                self._logger.warning(
                    "Unable to create client service: {}. This usually means the robot hasn't "
                    "finished booting yet. Will wait {} seconds and try again.".format(
                        e, sleep_secs
                    )
                )
                time.sleep(sleep_secs)

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._init_current_graph_nav_state()

        # Graphnav navigation initialization
        self._navigate_to_dynamic_feedback = ""
        self._graphnav_lock = threading.Lock()
        self._navigating = False

        # Default travel params for navigation
        velocity_x = 1  # m/s
        velocity_y = 1
        angular_velocity = 1  # radians/s
        max_dist = 1  # m
        max_yaw = 6.4  # radians
        velocity_max = geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=velocity_x, y=velocity_y),
            angular=angular_velocity,
        )
        velocity_min = geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=-velocity_x, y=-velocity_y),
            angular=-angular_velocity,
        )
        velocity_params = geometry_pb2.SE2VelocityLimit(
            max_vel=velocity_max, min_vel=velocity_min
        )
        self.graphnav_travel_params = self._graph_nav_client.generate_travel_params(
            max_dist, max_yaw, velocity_params
        )
        self._graphnav_paused = False
        self._graphnav_cancelled = False

        # Async Tasks
        self._async_task_list = []
        self._robot_state_task = AsyncRobotState(
            self._robot_state_client,
            self._logger,
            max(0.0, self._rates.get("robot_state", 0.0)),
            self._callbacks.get("robot_state", None),
        )
        self._robot_metrics_task = AsyncMetrics(
            self._robot_state_client,
            self._logger,
            max(0.0, self._rates.get("metrics", 0.0)),
            self._callbacks.get("metrics", None),
        )
        self._lease_task = AsyncLease(
            self._lease_client,
            self._logger,
            max(0.0, self._rates.get("lease", 0.0)),
            self._callbacks.get("lease", None),
        )
        self._idle_task = AsyncIdle(
            self._robot_command_client, self._logger, 10.0, self
        )
        self._estop_monitor = AsyncEStopMonitor(
            self._estop_client, self._logger, 20.0, self
        )

        self._estop_endpoint = None
        self._estop_keepalive = None

        robot_tasks = [
            self._robot_state_task,
            self._robot_metrics_task,
            self._lease_task,
            self._idle_task,
            self._estop_monitor,
        ]

        self._spot_check = SpotCheck(
            self._robot,
            self._logger,
            self._state,
            self._spot_check_client,
            self._robot_command_client,
            self._lease_client,
        )

        if self._robot.has_arm():
            self._spot_arm = SpotArm(
                self._robot,
                self._logger,
                self._state,
                self._robot_command_client,
                self._manipulation_api_client,
                self._robot_state_client,
                MAX_COMMAND_DURATION,
            )
        else:
            self._spot_arm = None

        self._spot_images = SpotImages(self._robot, self._logger, self._image_client)

        self._spot_docking = SpotDocking(
            self._robot,
            self._logger,
            self._state,
            self._command_data,
            self._docking_client,
            self._robot_command_client,
        )

        if self._point_cloud_client:
            self._spot_eap = SpotEAP(
                self._logger,
                self._point_cloud_client,
                point_cloud_sources,
                # If the parameter isn't given assume we don't want any clouds
                self._rates.get("point_cloud", 0.0),
                self._callbacks.get("lidar_points", None),
            )
            self._point_cloud_task = self._spot_eap.async_task
            robot_tasks.append(self._point_cloud_task)
        else:
            self._spot_eap = None

        self._spot_world_objects = SpotWorldObjects(
            self._logger,
            self._world_objects_client,
            self._rates.get("world_objects", 10),
            self._callbacks.get("world_objects", None),
        )
        self._world_objects_task = self._spot_world_objects.async_task
        robot_tasks.append(self._world_objects_task)

        self._async_tasks = AsyncTasks(robot_tasks)

        if self._is_licensed_for_choreography:
            self._spot_dance = SpotDance(
                self._robot, self._choreography_client, self._logger
            )

        self._robot_id = None
        self._lease = None

    @staticmethod
    def authenticate(
        robot: Robot, username: str, password: str, logger: logging.Logger
    ) -> bool:
        """
        Authenticate with a robot through the bosdyn API. A blocking function which will wait until authenticated (if
        the robot is still booting) or login fails

        Args:
            robot: Robot object which we are authenticating with
            username: Username to authenticate with
            password: Password for the given username
            logger: Logger with which to print messages

        Returns:
            boolean indicating whether authentication was successful
        """
        authenticated = False
        while not authenticated:
            try:
                logger.info("Trying to authenticate with robot...")
                robot.authenticate(username, password)
                robot.time_sync.wait_for_sync(10)
                logger.info("Successfully authenticated.")
                authenticated = True
            except RpcError as err:
                sleep_secs = 15
                logger.warn(
                    "Failed to communicate with robot: {}\nEnsure the robot is powered on and you can "
                    "ping {}. Robot may still be booting. Will retry in {} seconds".format(
                        err, robot.address, sleep_secs
                    )
                )
                time.sleep(sleep_secs)
            except bosdyn.client.auth.InvalidLoginError as err:
                logger.error("Failed to log in to robot: {}".format(err))
                raise err

        return authenticated

    @staticmethod
    def authenticate_from_payload_credentials(
        robot: Robot, payload_credentials_file: str, logger: logging.Logger
    ) -> bool:
        """
        Authenticate with a robot through the bosdyn API from payload credentials. A blocking function which will
        wait until authenticated (if the robot is still booting) or login fails

        Args:
            robot: Robot object which we are authenticating with
            payload_credentials_file: Path to the file to read payload credentials from
            logger: Logger with which to print messages

        Returns:

        """
        authenticated = False
        while not authenticated:
            try:
                logger.info(
                    "Trying to authenticate with robot from payload credentials..."
                )
                robot.authenticate_from_payload_credentials(
                    *bosdyn.client.util.read_payload_credentials(
                        payload_credentials_file
                    )
                )
                robot.time_sync.wait_for_sync(10)
                logger.info("Successfully authenticated.")
                authenticated = True
            except RpcError as err:
                sleep_secs = 15
                logger.warn(
                    "Failed to communicate with robot: {}\nEnsure the robot is powered on and you can "
                    "ping {}. Robot may still be booting. Will retry in {} seconds".format(
                        err, robot.address, sleep_secs
                    )
                )
                time.sleep(sleep_secs)
            except PayloadNotAuthorizedError as err:
                logger.error("Failed to authorize payload: {}".format(err))
                raise err

        return authenticated

    @property
    def robot_name(self) -> str:
        return self._robot_name

    @property
    def frame_prefix(self) -> str:
        return self._frame_prefix

    @property
    def spot_arm(self) -> SpotArm:
        """Return SpotArm instance"""
        if not self._robot.has_arm():
            raise MissingSpotArm()
        else:
            return self._spot_arm

    @property
    def spot_check(self) -> SpotCheck:
        """Return SpotCheck instance"""
        return self._spot_check

    @property
    def spot_eap_lidar(self) -> typing.Optional[SpotEAP]:
        """Return SpotEAP instance"""
        return self._spot_eap

    @property
    def logger(self) -> logging.Logger:
        """Return logger instance of the SpotWrapper"""
        return self._logger

    @property
    def is_valid(self) -> bool:
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._valid

    @property
    def id(self) -> str:
        """Return robot's ID"""
        return self._robot_id

    @property
    def robot_state(self) -> robot_state_pb2.RobotState:
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self) -> robot_state_pb2.RobotMetrics:
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def lease(self) -> typing.List[lease_pb2.LeaseResource]:
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def spot_images(self) -> SpotImages:
        """Return SpotImages instance"""
        return self._spot_images

    @property
    def spot_world_objects(self) -> SpotWorldObjects:
        """Return SpotWorldObjects instance"""
        return self._spot_world_objects

    @property
    def spot_docking(self) -> SpotDocking:
        """Return SpotDocking instance"""
        return self._spot_docking

    @property
    def world_objects(self) -> world_object_pb2.ListWorldObjectResponse:
        """Return most recent proto from _world_objects_task"""
        return self.spot_world_objects.async_task.proto

    @property
    def point_clouds(self) -> typing.List[point_cloud_pb2.PointCloudResponse]:
        """Return latest proto from the _point_cloud_task"""
        return self.spot_eap_lidar.async_task.proto

    @property
    def is_standing(self) -> bool:
        """Return boolean of standing state"""
        return self._state.is_standing

    @is_standing.setter
    def is_standing(self, state: bool) -> None:
        self._state.is_standing = state

    @property
    def is_sitting(self) -> bool:
        """Return boolean of standing state"""
        return self._state.is_sitting

    @is_sitting.setter
    def is_sitting(self, state: bool) -> None:
        self._state.is_sitting = state

    @property
    def is_moving(self) -> bool:
        """Return boolean of walking state"""
        return self._state.is_moving

    @is_moving.setter
    def is_moving(self, state: bool) -> None:
        self._state.is_moving = state

    @property
    def near_goal(self) -> bool:
        return self._state.near_goal

    @near_goal.setter
    def near_goal(self, state: bool) -> None:
        self._state.near_goal = state

    @property
    def at_goal(self) -> bool:
        return self._state.at_goal

    @at_goal.setter
    def at_goal(self, state: bool) -> None:
        self._state.at_goal = state

    @property
    def last_stand_command(self) -> typing.Optional[int]:
        return self._command_data.last_stand_command

    @last_stand_command.setter
    def last_stand_command(self, command_id: int) -> None:
        self._command_data.last_stand_command = command_id

    @property
    def last_sit_command(self) -> typing.Optional[int]:
        return self._command_data.last_sit_command

    @last_sit_command.setter
    def last_sit_command(self, command_id: int) -> None:
        self._command_data.last_sit_command = command_id

    @property
    def last_docking_command(self) -> typing.Optional[int]:
        return self._command_data.last_docking_command

    @last_docking_command.setter
    def last_docking_command(self, command_id: int) -> None:
        self._command_data.last_docking_command = command_id

    @property
    def last_trajectory_command(self) -> typing.Optional[int]:
        return self._command_data.last_trajectory_command

    @last_trajectory_command.setter
    def last_trajectory_command(self, command_id: int) -> None:
        self._command_data.last_trajectory_command = command_id

    @property
    def last_trajectory_command_precise(self) -> typing.Optional[bool]:
        return self._command_data.last_trajectory_command_precise

    @last_trajectory_command_precise.setter
    def last_trajectory_command_precise(self, was_precise: bool) -> None:
        self._command_data.last_trajectory_command_precise = was_precise

    @property
    def last_velocity_command_time(self) -> typing.Optional[float]:
        return self._command_data.last_velocity_command_time

    @last_velocity_command_time.setter
    def last_velocity_command_time(self, command_time: float) -> None:
        self._command_data.last_velocity_command_time = command_time

    def is_estopped(self, timeout: typing.Optional[float] = None) -> bool:
        return self._robot.is_estopped(timeout=timeout)

    def has_arm(self, timeout: typing.Optional[float] = None) -> bool:
        return self._robot.has_arm(timeout=timeout)

    @property
    def time_skew(self) -> Timestamp:
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def resetMobilityParams(self) -> None:
        """
        Resets the mobility parameters used for motion commands to the default values provided by the bosdyn api.
        Returns:
        """
        self._mobility_params = RobotCommandBuilder.mobility_params()

    def robotToLocalTime(self, timestamp: Timestamp) -> Timestamp:
        """Takes a timestamp and an estimated skew and return seconds and nano seconds in local time

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """
        return robotToLocalTime(timestamp, self._robot)

    def claim(self) -> typing.Tuple[bool, str]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        if self.lease is not None:
            for resource in self.lease:
                if (
                    resource.resource == "all-leases"
                    and SPOT_CLIENT_NAME in resource.lease_owner.client_name
                ):
                    return True, "We already claimed the lease"

        try:
            self._robot_id = self._robot.get_id()
            self.getLease()
            if self._start_estop and not self.check_is_powered_on():
                # If we are requested to start the estop, and the robot is not already powered on, then we reset the
                # estop. The robot already being powered on is relevant when the lease is being taken from someone
                # else who may already have the motor cut power authority - in this case we cannot take that
                # authority as the robot would have to sit down.
                self.resetEStop()
            return True, "Success"
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False, str(err)
        except Exception as err:
            self._logger.error(traceback.format_exc())
            return False, str(err)

    def updateTasks(self) -> None:
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            self._logger.error(f"Update tasks failed with error: {str(e)}")

    def resetEStop(self) -> None:
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, SPOT_CLIENT_NAME, self._estop_timeout
        )
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe: bool = True) -> typing.Tuple[bool, str]:
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_keepalive.stop()
            else:
                self._estop_keepalive.settle_then_cut()

            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to estop: {e}"

    def disengageEStop(self) -> typing.Tuple[bool, str]:
        """Disengages the E-Stop"""
        try:
            self._estop_keepalive.allow()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to disengage estop {e}"

    def releaseEStop(self) -> None:
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self) -> None:
        """Get a lease for the robot and keep the lease alive automatically."""
        if self._use_take_lease:
            self._lease = self._lease_client.take()
        else:
            self._lease = self._lease_client.acquire()

        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self) -> None:
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self) -> typing.Tuple[bool, str]:
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to release the lease: {e}"

    def disconnect(self) -> None:
        """Release control of robot as gracefully as posssible."""
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.releaseLease()
        self.releaseEStop()

    def _robot_command(
        self,
        command_proto: robot_command_pb2.RobotCommand,
        end_time_secs: typing.Optional[float] = None,
        timesync_endpoint: typing.Optional[TimeSyncEndpoint] = None,
    ) -> typing.Tuple[bool, str, typing.Optional[str]]:
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
            timesync_endpoint: (optional) Time sync endpoint

        Returns:
            Tuple of bool success, string message, and the command ID
        """
        try:
            command_id = self._robot_command_client.robot_command(
                lease=None,
                command=command_proto,
                end_time_secs=end_time_secs,
                timesync_endpoint=timesync_endpoint,
            )
            return True, "Success", command_id
        except Exception as e:
            self._logger.error(f"Unable to execute robot command: {e}")
            return False, str(e), None

    def _manipulation_request(
        self, request_proto, end_time_secs=None, timesync_endpoint=None
    ):
        """Generic function for sending requests to the manipulation api of a robot.

        Args:
            request_proto: manipulation_api_pb2 object to send to the robot.
        """
        try:
            command_id = self._manipulation_api_client.manipulation_api_command(
                manipulation_api_request=request_proto
            ).manipulation_cmd_id

            return True, "Success", command_id
        except Exception as e:
            self._logger.error(f"Unable to execute manipulation command: {e}")
            return False, str(e), None

    @try_claim
    def stop(self) -> typing.Tuple[bool, str]:
        """
        Stop any action the robot is currently doing.

        Returns:
            Tuple of bool success and a string message

        """
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    @try_claim(power_on=True)
    def self_right(self) -> typing.Tuple[bool, str]:
        """
        Have the robot self-right.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    @try_claim(power_on=True)
    def sit(self) -> typing.Tuple[bool, str]:
        """
        Stop the robot's motion and sit down if able.

        Returns:
            Tuple of bool success and a string message

        """
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self.last_sit_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def simple_stand(self, monitor_command: bool = True) -> typing.Tuple[bool, str]:
        """
        If the e-stop is enabled, and the motor power is enabled, stand the robot up.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(
            RobotCommandBuilder.synchro_stand_command(params=self._mobility_params)
        )
        if monitor_command:
            self.last_stand_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def stand(
        self,
        monitor_command: bool = True,
        body_height: float = 0,
        body_yaw: float = 0,
        body_pitch: float = 0,
        body_roll: float = 0,
    ) -> typing.Tuple[bool, str]:
        """
        If the e-stop is enabled, and the motor power is enabled, stand the robot up.
        Executes a stand command, but one where the robot will assume the pose specified by the given parameters.

        If no parameters are given this behaves just as a normal stand command

        Args:
            monitor_command: Track the state of the command in the async idle, which sets is_standing
            body_height: Offset of the body relative to normal stand height, in metres
            body_yaw: Yaw of the body in radians
            body_pitch: Pitch of the body in radians
            body_roll: Roll of the body in radians

        Returns:
            Tuple of bool success and a string message

        """
        if any([body_height, body_yaw, body_pitch, body_roll]):
            # If any of the orientation parameters are nonzero use them to pose the body
            body_orientation = EulerZXY(yaw=body_yaw, pitch=body_pitch, roll=body_roll)
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(
                    body_height=body_height, footprint_R_body=body_orientation
                )
            )
        else:
            # Otherwise just use the mobility params
            response = self._robot_command(
                RobotCommandBuilder.synchro_stand_command(params=self._mobility_params)
            )

        if monitor_command:
            self.last_stand_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def battery_change_pose(self, dir_hint: int = 1) -> typing.Tuple[bool, str]:
        """
        Put the robot into the battery change pose

        Args:
            dir_hint: 1 rolls to the right side of the robot, 2 to the left

        Returns:
            Tuple of bool success and a string message
        """
        if self.is_sitting:
            response = self._robot_command(
                RobotCommandBuilder.battery_change_pose_command(dir_hint)
            )
            return response[0], response[1]
        return False, "Call sit before trying to roll over"

    @try_claim
    def safe_power_off(self) -> typing.Tuple[bool, str]:
        """
        Stop the robot's motion and sit if possible.  Once sitting, disable motor power.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def clear_behavior_fault(
        self, fault_id: int
    ) -> typing.Tuple[bool, str, typing.Optional[bool]]:
        """
        Clear the behavior fault defined by the given id.

        Returns:
            Tuple of bool success, string message, and bool indicating whether the status was cleared
        """
        try:
            rid = self._robot_command_client.clear_behavior_fault(
                behavior_fault_id=fault_id, lease=None
            )
            return True, "Success", rid
        except Exception as e:
            return False, f"Exception while clearing behavior fault: {e}", None

    @try_claim
    def power_on(self) -> typing.Tuple[bool, str]:
        """
        Enable the motor power if e-stop is enabled.

        Returns:
            Tuple of bool success and a string message
        """
        # Don't bother trying to power on if we are already powered on
        if not self.check_is_powered_on():
            # If we are requested to start the estop, we have to acquire it when powering on.
            if self._start_estop:
                self.resetEStop()
            try:
                self._logger.info("Powering on")
                self._robot.power_on()
            except Exception as e:
                return False, f"Exception while powering on: {e}"

            return True, "Success"

        return True, "Was already powered on"

    def set_mobility_params(
        self, mobility_params: spot_command_pb2.MobilityParams
    ) -> None:
        """Set Params for mobility and movement

        Args:
            mobility_params: spot.MobilityParams, params for spot mobility commands.
        """
        self._mobility_params = mobility_params

    def get_mobility_params(self) -> spot_command_pb2.MobilityParams:
        """Get mobility params"""
        return self._mobility_params

    @try_claim
    def velocity_cmd(
        self, v_x: float, v_y: float, v_rot: float, cmd_duration: float = 0.125
    ) -> typing.Tuple[bool, str]:
        """

        Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).

        Returns:
            Tuple of bool success and a string message
        """
        end_time = time.time() + cmd_duration
        response = self._robot_command(
            RobotCommandBuilder.synchro_velocity_command(
                v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params
            ),
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )
        self.last_velocity_command_time = end_time
        return response[0], response[1]

    @try_claim
    def trajectory_cmd(
        self,
        goal_x: float,
        goal_y: float,
        goal_heading: float,
        cmd_duration: float,
        frame_name: str = "odom",
        precise_position: bool = False,
        mobility_params: spot_command_pb2.MobilityParams = None,
    ) -> typing.Tuple[bool, str]:
        """Send a trajectory motion command to the robot.

        Args:
            goal_x: Position X coordinate in meters
            goal_y: Position Y coordinate in meters
            goal_heading: Pose heading in radians
            cmd_duration: Time-to-live for the command in seconds.
            frame_name: frame_name to be used to calc the target position. 'odom' or 'vision'
            precise_position: if set to false, the status STATUS_NEAR_GOAL and STATUS_AT_GOAL will be equivalent. If
            true, the robot must complete its final positioning before it will be considered to have successfully
            reached the goal.
            mobility_params: Mobility parameters to send along with this command

        Returns:
            (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        if mobility_params is None:
            mobility_params = self._mobility_params
        self._trajectory_status_unknown = False
        self.at_goal = False
        self.near_goal = False
        self.last_trajectory_command_precise = precise_position
        self._logger.info("got command duration of {}".format(cmd_duration))
        end_time = time.time() + cmd_duration
        if frame_name == "vision":
            vision_tform_body = frame_helpers.get_vision_tform_body(
                self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            vision_tform_goal = vision_tform_body * body_tform_goal
            response = self._robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=vision_tform_goal.x,
                    goal_y=vision_tform_goal.y,
                    goal_heading=vision_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.VISION_FRAME_NAME,
                    params=mobility_params,
                ),
                end_time_secs=end_time,
            )
        elif frame_name == "odom":
            odom_tform_body = frame_helpers.get_odom_tform_body(
                self._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            )
            body_tform_goal = math_helpers.SE3Pose(
                x=goal_x, y=goal_y, z=0, rot=math_helpers.Quat.from_yaw(goal_heading)
            )
            odom_tform_goal = odom_tform_body * body_tform_goal
            response = self._robot_command(
                RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    goal_x=odom_tform_goal.x,
                    goal_y=odom_tform_goal.y,
                    goal_heading=odom_tform_goal.rot.to_yaw(),
                    frame_name=frame_helpers.ODOM_FRAME_NAME,
                    params=mobility_params,
                ),
                end_time_secs=end_time,
            )
        else:
            raise ValueError("frame_name must be 'vision' or 'odom'")
        if response[0]:
            self.last_trajectory_command = response[2]
        return response[0], response[1]

    def robot_command(
        self, robot_command: robot_command_pb2.RobotCommand
    ) -> typing.Tuple[bool, str]:
        end_time = time.time() + MAX_COMMAND_DURATION
        return self._robot_command(
            robot_command,
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )

    def manipulation_command(self, request):
        end_time = time.time() + MAX_COMMAND_DURATION
        return self._manipulation_request(
            request,
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )

    def get_robot_command_feedback(
        self, cmd_id: int
    ) -> robot_command_pb2.RobotCommandFeedbackResponse:
        return self._robot_command_client.robot_command_feedback(cmd_id)

    def get_manipulation_command_feedback(self, cmd_id):
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_id
        )

        return self._manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request
        )

    def list_graph(self, upload_path=None):
        """List waypoint ids of garph_nav
        Args:
          upload_path : Path to the root directory of the map.
        """
        ids, eds = self._list_graph_waypoint_and_edge_ids()
        # skip waypoint_ for v2.2.1, skip waypiont for < v2.2
        return [
            v
            for k, v in sorted(
                ids.items(), key=lambda id: int(id[0].replace("waypoint_", ""))
            )
        ]

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

    @try_claim
    def navigate_to(
        self,
        upload_path,
        navigate_to,
        initial_localization_fiducial=True,
        initial_localization_waypoint=None,
    ):
        """navigate with graph nav.

        Args:
           upload_path : Path to the root directory of the map.
           navigate_to : Waypont id string for where to goal
           initial_localization_fiducial : Tells the initializer whether to use fiducials
           initial_localization_waypoint : Waypoint id string of current robot position (optional)
        """

        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path[-1] == "/":
            upload_filepath = upload_path[:-1]
        else:
            upload_filepath = upload_path

        # Boolean indicating the robot's power state.
        power_state = self._robot_state_client.get_robot_state().power_state
        self._started_powered_on = power_state.motor_power_state == power_state.STATE_ON
        self._powered_on = self._started_powered_on

        # FIX ME somehow,,,, if the robot is stand, need to sit the robot before starting garph nav
        if self.is_standing and not self.is_moving:
            self.sit()

        # TODO verify estop  / claim / power_on
        self._clear_graph()
        self._upload_graph_and_snapshots(upload_filepath)
        if initial_localization_fiducial:
            self._set_initial_localization_fiducial()
        if initial_localization_waypoint:
            self._set_initial_localization_waypoint([initial_localization_waypoint])
        self._list_graph_waypoint_and_edge_ids()
        self._get_localization_state()
        resp = self._navigate_to([navigate_to])

        return resp

    @try_claim
    def navigate_to_dynamic(self, navigate_to):
        """Navigate with graph nav. Use this instead of navigate_to if you want to modify Spot's speed during navigation commands
         and pause/cancel navigations (using set_navigate_to_params). Unlike navigate_to, this method assumes
         the graphnav map has already been uploaded and spot has been localized. This results in reduced latency between calling
         this method and Spot starting navigation

        Args:
           navigate_to : Waypont id string for where to goal
        """
        self._get_localization_state()
        resp = self._navigate_to_dynamic([navigate_to])

        return resp
    
    def set_navigate_to_params(
        self, x: float, y: float, max_dist: float, max_yaw: float
    ) -> bool:
        """Change the navigation params used by navigate_to_dynamic or pause/cancel navigation

       Args:
            x: the maximum speed (m/s) for Spot in the x (forwards) direction
            y: the maximum speed (m/s) for Spot in the y (sideways) direction
            max_dist: Threshold for acceptable distance (in m) between Spot and the goal waypoint
            max_yaw: Threashold for the acceptable angle (in radians) between Spot and the goal waypoint

        Note that if this method is not used, the default values are (1, 1, 1, 6.4).
        Setting x or y to 0 will pause the current navigation till x and y are both non-zero
        Setting x or y to a negative value will cancel the current navigation and will cause any ongoing
        navigate_to_dynamic call to return
        """

        self._graphnav_lock.acquire()
        try:
            # Pause graphnav if x or y is zero
            if x == 0 or y == 0:
                self._graphnav_paused = True
            else:
                self._graphnav_paused = False
            
            # cancel graphnav if either value is negative
            if (x < 0 or y < 0):
                self._graphnav_cancelled = True
            else:
                self._graphnav_cancelled = False
            if self._graphnav_cancelled:
                # release the lock, wait for the graphnav to terminate, and then return
                # this makes sure a cancelation request can not be overidden by future set_nav_param requests
                # Note that requesting a cancellation while spot is not navigating has no effect,
                # and future navigate_to_dynamic requests will run normally
                self._graphnav_lock.release()
                while self._navigating:
                    time.sleep(0.2)
                return True
                    
            # hardcode the angular_velocity limit
            angular_velocity_limit = 1
            velocity_max = geometry_pb2.SE2Velocity(linear = geometry_pb2.Vec2(x = x, y = y), angular = angular_velocity_limit)
            velocity_min = geometry_pb2.SE2Velocity(linear = geometry_pb2.Vec2(x = - x, y = - y), angular = -angular_velocity_limit)
            velocity_params = geometry_pb2.SE2VelocityLimit(max_vel = velocity_max, min_vel = velocity_min)
            self.graphnav_travel_params = self._graph_nav_client.generate_travel_params(max_dist, max_yaw, velocity_params)

            # The code upto this point should be sufficient to modify velocity
            # However due to a SDK bug, we need to change the goal waypoint temporarily for Graphnav to register the change in travel params.
            # Have talked to this BD support and they confirmed this is a bug.

            # If spot is navigating, we will send a quick navigation request to its current location (not noticable)
            if self._navigating:
                localization_state = self._graph_nav_client.get_localization_state()
                current_waypoint = localization_state.localization.waypoint_id
                if not current_waypoint: # Should be able to get current waypoint, something has gone wrong
                    self._graphnav_lock.release()
                    return False
                self._graph_nav_client.navigate_to(
                        current_waypoint, 0.1,  leases=[self.navigate_to_dynamic_sublease.lease_proto])
                time.sleep(0.1) #Sleep to wait for navigation to complete
            self._graphnav_lock.release()
            return True
        except Exception:
            # Release lock to avoid deadlocks in future runs
            self._graphnav_lock.release() 
            raise

    def _init_current_graph_nav_state(self):
        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  # maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        self._current_annotation_name_to_wp_id = dict()
        self._current_anchored_world_objects = (
            dict()
        )  # maps object id to a (wo, waypoint, fiducial)
        self._current_anchors = dict()  # maps anchor id to anchor

    ## copy from spot-sdk/python/examples/graph_nav_command_line/graph_nav_command_line.py
    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        self._logger.info("Got localization: \n%s" % str(state.localization))
        odom_tform_body = get_odom_tform_body(
            state.robot_kinematics.transforms_snapshot
        )
        self._logger.info(
            "Got robot state in kinematic odometry frame: \n%s" % str(odom_tform_body)
        )

    def _set_initial_localization_fiducial(self, *args):
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

    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            self._logger.error("No waypoint specified to initialize to.")
            return
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
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

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""

        # Download current graph
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self._logger.error("Empty graph.")
            return
        self._current_graph = graph

        localization_id = (
            self._graph_nav_client.get_localization_state().localization.waypoint_id
        )

        # Update and print waypoints and edges
        (
            self._current_annotation_name_to_wp_id,
            self._current_edges,
        ) = graph_nav_util.update_waypoints_and_edges(
            graph, localization_id, self._logger
        )
        return self._current_annotation_name_to_wp_id, self._current_edges

    def _upload_graph_and_snapshots(self, upload_filepath):
        """Upload the graph and snapshots to the robot."""
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
            self.getLease()
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
                "Upload complete! The robot is currently not localized to the map; "
                "please localize the robot"
            )

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
                self.logger.warn(
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
                self.logger.warn(
                    "Failed to download edge snapshot: %s", edge.snapshot_id
                )
                continue
            self._write_bytes_while_download(
                os.path.join(download_path, "edge_snapshots", edge.snapshot_id),
                edge_snapshot.SerializeToString(),
            )
        return True, "Success"

    @try_claim
    def _navigate_to(self, *args):
        """Navigate to a specific waypoint."""
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self._logger.info("No waypoint provided as a destination for navigate to.")
            return
        self._lease = self._lease_wallet.get_lease()
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )
        if not destination_waypoint:
            # Failed to find the appropriate unique waypoint id for the navigation command.
            return
        if not self.toggle_power(should_power_on=True):
            self._logger.info(
                "Failed to power on the robot, and cannot complete navigate to request."
            )
            return

        # Stop the lease keepalive and create a new sublease for graph nav.
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
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

        # Update the lease and power off the robot if appropriate.
        if self._powered_on and not self._started_powered_on:
            # Sit the robot down + power off after the navigation command is complete.
            self.toggle_power(should_power_on=False)

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

    @try_claim
    def _navigate_to_dynamic(self, *args):
        """Navigate to a specific waypoint. Can change velocity and pause/cancel navigation (see set_navigate_to_params)"""

        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self._logger.info("No waypoint provided as a destination for navigate to.")
            return False, "no goal waypoint provided"

        self._lease = self._lease_wallet.get_lease()
        destination_waypoint = graph_nav_util.find_unique_waypoint_id(
            args[0][0],
            self._current_graph,
            self._current_annotation_name_to_wp_id,
            self._logger,
        )

        if not destination_waypoint:
            self._logger.info("Could not find destination waypoint")
            return False, "could not find destination waypoint"

        if not self.toggle_power(should_power_on=True):
            self._logger.info(
                "Failed to power on the robot, and cannot complete navigate to request."
            )
            return False, "failed to power on robot"

        # Stop the lease keepalive and create a new sublease for graph nav.
        self._lease = self._lease_wallet.advance()
        self.navigate_to_dynamic_sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Can't cancel a navigation before it starts
        self._graphnav_cancelled = False
        # Navigate to the destination waypoint.
        is_finished = False
        nav_to_cmd_id = -1
        try:
            while not is_finished:
                # Acquire lock to coordinate with set_navigate_to_params
                self._graphnav_lock.acquire()
                self._navigating = True
                # Navigate to the destination waypoint.
                if self._graphnav_cancelled:
                    self._navigate_to_dynamic_feedback = "goal cancelled"
                    self._navigating = False
                    self._graphnav_lock.release()
                    return False, "goal was cancelled"

                if self._graphnav_paused:
                    self._navigate_to_dynamic_feedback = "goal paused"
                    self._graphnav_lock.release()
                    time.sleep(0.5)
                    continue

                # Issue the navigation command about twice a second so that user can update velocity/cancel/pause
                nav_to_cmd_id = self._graph_nav_client.navigate_to(
                    destination_waypoint,
                    1.0,
                    leases=[self.navigate_to_dynamic_sublease.lease_proto],
                    travel_params=self.graphnav_travel_params,
                )
                self._graphnav_lock.release()
                time.sleep(
                    0.5
                )  # Sleep for half a second to allow for command execution.

                # Poll the robot for feedback to determine if the navigation command is complete.
                is_finished = self._check_success(nav_to_cmd_id)

        except Exception:
            # In case navigation errors, release lock to avoid deadlocks in successive runs
            self._graphnav_lock.release()
            raise

        self._navigating = False
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

    @try_claim
    def _navigate_route(self, *args):
        """Navigate through a specific route of waypoints."""
        if len(args) < 1:
            # If no waypoint ids are given as input, then return without requesting navigation.
            self._logger.error("No waypoints provided for navigate route.")
            return
        waypoint_ids = args[0]
        for i in range(len(waypoint_ids)):
            waypoint_ids[i] = graph_nav_util.find_unique_waypoint_id(
                waypoint_ids[i],
                self._current_graph,
                self._current_annotation_name_to_wp_id,
                self._logger,
            )
            if not waypoint_ids[i]:
                # Failed to find the unique waypoint id.
                return

        edge_ids_list = []
        all_edges_found = True
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                all_edges_found = False
                self._logger.error(
                    "Failed to find an edge between waypoints: {} and {}.".format(
                        start_wp, end_wp
                    )
                )
                self._logger.error(
                    "List the graph's waypoints and edges to ensure pairs of waypoints has an edge."
                )
                break

        self._lease = self._lease_wallet.get_lease()
        if all_edges_found:
            if not self.toggle_power(should_power_on=True):
                self._logger.error(
                    "Failed to power on the robot, and cannot complete navigate route request."
                )
                return

            # Stop the lease keepalive and create a new sublease for graph nav.
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
                time.sleep(
                    0.5
                )  # Sleep for half a second to allow for command execution.
                # Poll the robot for feedback to determine if the route is complete. Then sit
                # the robot down once it is finished.
                is_finished = self._check_success(nav_route_command_id)

            self._lease = self._lease_wallet.advance()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)

            # Update the lease and power off the robot if appropriate.
            if self._powered_on and not self._started_powered_on:
                # Sit the robot down + power off after the navigation command is complete.
                self.toggle_power(should_power_on=False)

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges in the RAM of the robot."""
        result = self._graph_nav_client.clear_graph(lease=self._lease.lease_proto)
        self._init_current_graph_nav_state()
        return result

    @try_claim
    def toggle_power(self, should_power_on):
        """Power the robot on/off dependent on the current power state."""
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._robot_state_client.get_robot_state_async()
                state_response = future.result(
                    timeout=10
                )  # 10 second timeout for waiting for the state response.
                if (
                    state_response.power_state.motor_power_state
                    == robot_state_pb2.PowerState.STATE_ON
                ):
                    motors_on = True
                else:
                    # Motors are not yet fully powered on.
                    time.sleep(0.25)
        elif is_powered_on and not should_power_on:
            # Safe power off (robot will sit then power down) when it is in a
            # powered-on state.
            safe_power_off(self._robot_command_client, self._robot_state_client)
        else:
            # Return the current power state without change.
            return is_powered_on
        # Update the locally stored power state.
        self.check_is_powered_on()
        return self._powered_on

    def check_is_powered_on(self) -> bool:
        """Determine if the robot is powered on or off."""
        power_state = self._robot_state_client.get_robot_state().power_state
        self._powered_on = power_state.motor_power_state == power_state.STATE_ON
        return self._powered_on

    def _check_success(self, command_id=-1):
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

    def _match_edge(self, current_edges, waypoint1, waypoint2):
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

    @try_claim
    def execute_dance(self, data):
        if self._is_licensed_for_choreography:
            return self._spot_dance.execute_dance(data)
        else:
            return False, "Spot is not licensed for choreography"

    def upload_animation(
        self, animation_name: str, animation_file_content: str
    ) -> typing.Tuple[bool, str]:
        if self._is_licensed_for_choreography:
            return self._spot_dance.upload_animation(
                animation_name, animation_file_content
            )
        else:
            return False, "Spot is not licensed for choreography"

    def list_all_moves(self) -> typing.Tuple[bool, str, typing.List[str]]:
        if self._is_licensed_for_choreography:
            return self._spot_dance.list_all_moves()
        else:
            return False, "Spot is not licensed for choreography", []

    def list_all_dances(self) -> typing.Tuple[bool, str, typing.List[str]]:
        if self._is_licensed_for_choreography:
            return self._spot_dance.list_all_dances()
        else:
            return False, "Spot is not licensed for choreography", []

    def get_docking_state(self, **kwargs):
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state
