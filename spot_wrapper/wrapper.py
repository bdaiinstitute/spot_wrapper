import functools
import logging
import time
import traceback
import typing

import bosdyn.client.auth
from bosdyn.api import image_pb2
from bosdyn.api import lease_pb2
from bosdyn.api import point_cloud_pb2
from bosdyn.api import robot_command_pb2
from bosdyn.api import robot_state_pb2
from bosdyn.api import world_object_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client import frame_helpers
from bosdyn.client import math_helpers
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import (
    EstopClient,
    EstopEndpoint,
    EstopKeepAlive,
)
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.license import LicenseClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot import UnregisteredServiceError
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.spot_check import SpotCheckClient
from bosdyn.client.time_sync import TimeSyncEndpoint
from bosdyn.client.world_object import WorldObjectClient

try:
    from bosdyn.choreography.client.choreography import (
        ChoreographyClient,
    )
    from .spot_dance import SpotDance

    HAVE_CHOREOGRAPHY_MODULE = True
except ModuleNotFoundError:
    HAVE_CHOREOGRAPHY_MODULE = False

from bosdyn.geometry import EulerZXY

from bosdyn.api import basic_command_pb2
from google.protobuf.timestamp_pb2 import Timestamp

from .spot_arm import SpotArm
from .spot_world_objects import SpotWorldObjects
from .spot_eap import SpotEAP
from .spot_docking import SpotDocking
from .spot_graph_nav import SpotGraphNav
from .spot_check import SpotCheck
from .spot_images import SpotImages

SPOT_CLIENT_NAME = "ros_spot"
MAX_COMMAND_DURATION = 1e5

"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"


def robotToLocalTime(timestamp, robot):
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
        rtime.nanos = rtime.nanos + 1000000000
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
    """Class to check if the robot is moving, and if not, command a stand with the set mobility parameters

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        spot_wrapper: A handle to the wrapper library
    """

    def __init__(self, client, logger, rate, spot_wrapper):
        super(AsyncIdle, self).__init__("idle", client, logger, period_sec=1.0 / rate)

        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if self._spot_wrapper._last_stand_command != None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_stand_command
                )
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status
                )
                self._spot_wrapper._robot_params["is_sitting"] = False
                if status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                    self._spot_wrapper._robot_params["is_standing"] = True
                    self._spot_wrapper._last_stand_command = None
                elif (
                    status == basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS
                ):
                    self._spot_wrapper._robot_params["is_standing"] = False
                else:
                    self._logger.warning("Stand command in unknown state")
                    self._spot_wrapper._robot_params["is_standing"] = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_stand_command = None

        if self._spot_wrapper._last_sit_command != None:
            try:
                self._spot_wrapper._robot_params["is_standing"] = False
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_sit_command
                )
                if (
                    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status
                    == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING
                ):
                    self._spot_wrapper._robot_params["is_sitting"] = True
                    self._spot_wrapper._last_sit_command = None
                else:
                    self._spot_wrapper._robot_params["is_sitting"] = False
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_sit_command = None

        is_moving = False

        if self._spot_wrapper._last_velocity_command_time != None:
            if time.time() < self._spot_wrapper._last_velocity_command_time:
                is_moving = True
            else:
                self._spot_wrapper._last_velocity_command_time = None

        if self._spot_wrapper._last_trajectory_command != None:
            try:
                response = self._client.robot_command_feedback(
                    self._spot_wrapper._last_trajectory_command
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
                        and not self._spot_wrapper._last_trajectory_command_precise
                    )
                ):
                    self._spot_wrapper._robot_params["at_goal"] = True
                    # Clear the command once at the goal
                    self._spot_wrapper._last_trajectory_command = None
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
                    self._spot_wrapper._robot_params["near_goal"] = True
                elif (
                    status
                    == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_UNKNOWN
                ):
                    self._spot_wrapper._trajectory_status_unknown = True
                    self._spot_wrapper._last_trajectory_command = None
                else:
                    self._logger.error(
                        "Received trajectory command status outside of expected range, value is {}".format(
                            status
                        )
                    )
                    self._spot_wrapper._last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper._last_trajectory_command = None

        self._spot_wrapper._robot_params["is_moving"] = is_moving

        # We must check if any command currently has a non-None value for its id. If we don't do this, this stand
        # command can cause other commands to be interrupted before they get to start
        if (
            self._spot_wrapper._robot_params["is_standing"]
            and self._spot_wrapper._continually_try_stand
            and not self._spot_wrapper._robot_params["is_moving"]
            and self._spot_wrapper._last_trajectory_command is not None
            and self._spot_wrapper._last_stand_command is not None
            and self._spot_wrapper._last_velocity_command_time is not None
            and self._spot_wrapper._last_docking_command is not None
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
    ):
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
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._at_goal = False
        self._near_goal = False
        self._trajectory_status_unknown = False
        self._last_robot_command_feedback = False
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None
        self._last_docking_command = None

        try:
            self._sdk = create_standard_sdk(SPOT_CLIENT_NAME)
        except Exception as e:
            self._logger.error(f"Error creating SDK object: {e}")
            self._valid = False
            return
        if HAVE_CHOREOGRAPHY_MODULE:
            self._sdk.register_service_client(ChoreographyClient)

        self._logger.info("Initialising robot at {}".format(self._hostname))
        self._robot = self._sdk.create_robot(self._hostname)

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

        # Clients
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
                self._map_processing_client = self._robot.ensure_client(
                    MapProcessingServiceClient.default_service_name
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
                try:
                    self._point_cloud_client = self._robot.ensure_client(
                        VELODYNE_SERVICE_NAME
                    )
                except UnregisteredServiceError as e:
                    self._point_cloud_client = None
                    self._logger.info("Velodyne point cloud service is not available.")

                self._license_client = self._robot.ensure_client(
                    LicenseClient.default_service_name
                )

                if HAVE_CHOREOGRAPHY_MODULE:
                    if self._license_client.get_feature_enabled(
                        [ChoreographyClient.license_name]
                    )[ChoreographyClient.license_name]:
                        self._sdk.register_service_client(ChoreographyClient)
                        self._choreography_client = self._robot.ensure_client(
                            ChoreographyClient.default_service_name
                        )
                        self._is_licensed_for_choreography = True
                    else:
                        self._logger.info(
                            f"Robot is not licensed for choreography: {e}"
                        )
                        self._is_licensed_for_choreography = False
                        self._choreography_client = None
                else:
                    self._logger.info(f"Choreography is not available.")
                    self._is_licensed_for_choreography = False
                    self._choreography_client = None

                if self._robot.has_arm():
                    self._manipulation_api_client = self._robot.ensure_client(
                        ManipulationApiClient.default_service_name
                    )
                else:
                    self._manipulation_api_client = None
                    self._logger.info("Manipulation API is not available.")

                self._robot_clients = {
                    "robot_state_client": self._robot_state_client,
                    "robot_command_client": self._robot_command_client,
                    "graph_nav_client": self._graph_nav_client,
                    "map_processing_client": self._map_processing_client,
                    "power_client": self._power_client,
                    "lease_client": self._lease_client,
                    "image_client": self._image_client,
                    "estop_client": self._estop_client,
                    "docking_client": self._docking_client,
                    "spot_check_client": self._spot_check_client,
                    "robot_command_method": self._robot_command,
                    "world_objects_client": self._world_objects_client,
                    "manipulation_api_client": self._manipulation_api_client,
                    "choreography_client": self._choreography_client,
                    "point_cloud_client": self._point_cloud_client,
                }

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

        # Core Async Tasks
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
        self._robot_id = None
        self._lease = None

        robot_tasks = [
            self._robot_state_task,
            self._robot_metrics_task,
            self._lease_task,
            self._idle_task,
            self._estop_monitor,
        ]

        # Create component objects for different functionality
        self._robot_params = {
            "is_standing": self._is_standing,
            "is_sitting": self._is_sitting,
            "is_moving": self._is_moving,
            "at_goal": self._at_goal,
            "near_goal": self._near_goal,
            "robot_id": self._robot_id,
            "estop_timeout": self._estop_timeout,
            "rates": self._rates,
            "callbacks": self._callbacks,
        }
        self.spot_image = SpotImages(
            self._robot, self._logger, self._robot_params, self._robot_clients
        )

        if self._robot.has_arm():
            self._spot_arm = SpotArm(
                self._robot,
                self._logger,
                self._robot_params,
                self._robot_clients,
                MAX_COMMAND_DURATION,
            )
            self._hand_image_task = self._spot_arm.hand_image_task
            robot_tasks.append(self._hand_image_task)
        else:
            self._spot_arm = None

        self._spot_docking = SpotDocking(
            self._robot, self._logger, self._robot_params, self._robot_clients
        )
        self._spot_graph_nav = SpotGraphNav(
            self._robot, self._logger, self._robot_params, self._robot_clients
        )
        self._spot_check = SpotCheck(
            self._robot, self._logger, self._robot_params, self._robot_clients
        )
        self._spot_images = SpotImages(
            self._robot,
            self._logger,
            self._robot_params,
            self._robot_clients,
            self._rgb_cameras,
        )

        if self._point_cloud_client:
            self._spot_eap = SpotEAP(
                self._robot, self._logger, self._robot_params, self._robot_clients
            )
            self._point_cloud_task = self._spot_eap.async_task
            robot_tasks.append(self._point_cloud_task)
        else:
            self._spot_eap = None

        self._spot_world_objects = SpotWorldObjects(
            self._robot, self._logger, self._robot_params, self._robot_clients
        )
        self._world_objects_task = self._spot_world_objects.async_task
        robot_tasks.append(self._world_objects_task)

        if self._is_licensed_for_choreography:
            self._spot_dance = SpotDance(
                self._robot, self._choreography_client, self._logger
            )

        self._async_tasks = AsyncTasks(robot_tasks)

    @staticmethod
    def authenticate(robot, username, password, logger):
        """
        Authenticate with a robot through the bosdyn API. A blocking function which will wait until authenticated (if
        the robot is still booting) or login fails

        Args:
            robot: Robot object which we are authenticating with
            username: Username to authenticate with
            password: Password for the given username
            logger: Logger with which to print messages

        Returns:

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

    @property
    def robot_name(self) -> str:
        return self._robot_name

    @property
    def frame_prefix(self) -> str:
        return self._frame_prefix

    @property
    def spot_images(self) -> SpotImages:
        """Return SpotImages instance"""
        return self._spot_images

    @property
    def spot_arm(self) -> SpotArm:
        """Return SpotArm instance"""
        if not self._robot.has_arm():
            raise MissingSpotArm()
        else:
            return self._spot_arm

    @property
    def spot_eap_lidar(self) -> SpotEAP:
        """Return SpotEAP instance"""
        return self._spot_eap

    @property
    def spot_world_objects(self) -> SpotWorldObjects:
        """Return SpotWorldObjects instance"""
        return self._spot_world_objects

    @property
    def spot_docking(self) -> SpotDocking:
        """Return SpotDocking instance"""
        return self._spot_docking

    @property
    def spot_graph_nav(self) -> SpotGraphNav:
        """Return SpotGraphNav instance"""
        return self._spot_graph_nav

    @property
    def spot_check(self) -> SpotCheck:
        """Return SpotCheck instance"""
        return self._spot_check

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
    def world_objects(self) -> world_object_pb2.ListWorldObjectResponse:
        """Return most recent proto from _world_objects_task"""
        return self.spot_world_objects.async_task.proto

    @property
    def hand_images(self) -> typing.List[image_pb2.ImageResponse]:
        """Return latest proto from the _hand_image_task"""
        return self.spot_arm.hand_image_task.proto

    @property
    def point_clouds(self) -> typing.List[point_cloud_pb2.PointCloudResponse]:
        """Return latest proto from the _point_cloud_task"""
        return self.spot_eap_lidar.async_task.proto

    @property
    def is_standing(self) -> bool:
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def is_sitting(self) -> bool:
        """Return boolean of standing state"""
        return self._is_sitting

    @property
    def is_moving(self) -> bool:
        """Return boolean of walking state"""
        return self._is_moving

    @property
    def near_goal(self) -> bool:
        return self._near_goal

    @property
    def at_goal(self) -> bool:
        return self._at_goal

    def is_estopped(self, timeout=None) -> bool:
        return self._robot.is_estopped(timeout=timeout)

    def has_arm(self, timeout=None):
        return self._robot.has_arm(timeout=timeout)

    @property
    def time_skew(self) -> Timestamp:
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def resetMobilityParams(self):
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
            self._logger.error(traceback.format_exc(), flush=True)
            return False, str(err)

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        try:
            self._async_tasks.update()
        except Exception as e:
            self._logger.error(f"Update tasks failed with error: {str(e)}")

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, SPOT_CLIENT_NAME, self._estop_timeout
        )
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True):
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

    def disengageEStop(self):
        """Disengages the E-Stop"""
        try:
            self._estop_keepalive.allow()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to disengage estop {e}"

    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self):
        """Get a lease for the robot and keep the lease alive automatically."""
        if self._use_take_lease:
            self._lease = self._lease_client.take()
        else:
            self._lease = self._lease_client.acquire()

        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def releaseLease(self):
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self):
        """Return the lease on the body and the eStop handle."""
        try:
            self.releaseLease()
            self.releaseEStop()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to release the lease: {e}"

    def disconnect(self):
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

    @try_claim
    def stop(self):
        """Stop the robot's motion."""
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    @try_claim(power_on=True)
    def self_right(self):
        """Have the robot self-right itself."""
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    @try_claim(power_on=True)
    def sit(self):
        """Stop the robot's motion and sit down if able."""
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def simple_stand(self, monitor_command=True):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        response = self._robot_command(
            RobotCommandBuilder.synchro_stand_command(params=self._mobility_params)
        )
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def stand(
        self, monitor_command=True, body_height=0, body_yaw=0, body_pitch=0, body_roll=0
    ):
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
            self._last_stand_command = response[2]
        return response[0], response[1]

    @try_claim(power_on=True)
    def battery_change_pose(self, dir_hint: int = 1):
        """
        Put the robot into the battery change pose

        Args:
            dir_hint: 1 rolls to the right side of the robot, 2 to the left
        """
        if self._is_sitting:
            response = self._robot_command(
                RobotCommandBuilder.battery_change_pose_command(dir_hint)
            )
            return response[0], response[1]
        return False, "Call sit before trying to roll over"

    @try_claim
    def safe_power_off(self):
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def clear_behavior_fault(self, id):
        """Clear the behavior fault defined by id."""
        try:
            rid = self._robot_command_client.clear_behavior_fault(
                behavior_fault_id=id, lease=None
            )
            return True, "Success", rid
        except Exception as e:
            return False, f"Exception while clearing behavior fault: {e}", None

    @try_claim
    def power_on(self):
        """Enble the motor power if e-stop is enabled."""
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

    def set_mobility_params(self, mobility_params: spot_command_pb2.MobilityParams):
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
        self,
        v_x: float,
        v_y: float,
        v_rot: float,
        cmd_duration: float = 0.7,
    ) -> typing.Tuple[bool, str]:
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        end_time = time.time() + cmd_duration
        response = self._robot_command(
            RobotCommandBuilder.synchro_velocity_command(
                v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params
            ),
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )
        self._last_velocity_command_time = end_time
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

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        if mobility_params is None:
            mobility_params = self._mobility_params
        self._trajectory_status_unknown = False
        self._at_goal = False
        self._near_goal = False
        self._last_trajectory_command_precise = precise_position
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
            self._last_trajectory_command = response[2]
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

    def get_robot_command_feedback(
        self, cmd_id: int
    ) -> robot_command_pb2.RobotCommandFeedbackResponse:
        return self._robot_command_client.robot_command_feedback(cmd_id)

    def check_is_powered_on(self):
        """Determine if the robot is powered on or off."""
        power_state = self._robot_state_client.get_robot_state().power_state
        self._powered_on = power_state.motor_power_state == power_state.STATE_ON
        return self._powered_on

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
