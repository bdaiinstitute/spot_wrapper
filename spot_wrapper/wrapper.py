import logging
import time
import traceback
import typing

import bosdyn.client.auth
from bosdyn.api import (
    basic_command_pb2,
    lease_pb2,
    manipulation_api_pb2,
    point_cloud_pb2,
    robot_command_pb2,
    robot_state_pb2,
    world_object_pb2,
)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    ChoreographySequence,
    ChoreographyStatusResponse,
    StartRecordingStateResponse,
    StopRecordingStateResponse,
    UploadChoreographyResponse,
)
from bosdyn.choreography.client.choreography import (
    ChoreographyClient,
)
from bosdyn.client import (
    ResponseError,
    RpcError,
    create_standard_sdk,
    frame_helpers,
    math_helpers,
)
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.docking import DockingClient
from bosdyn.client.estop import (
    EstopClient,
    EstopEndpoint,
    EstopKeepAlive,
)
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import Lease
from bosdyn.client.license import LicenseClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.payload_registration import PayloadNotAuthorizedError
from bosdyn.client.power import PowerClient, power_on, safe_power_off
from bosdyn.client.robot import Robot, UnregisteredServiceError
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_sit
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.spot_check import SpotCheckClient
from bosdyn.client.time_sync import TimeSyncEndpoint
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.geometry import EulerZXY
from bosdyn.mission.client import MissionClient
from google.protobuf.timestamp_pb2 import Timestamp

from .spot_arm import SpotArm
from .spot_check import SpotCheck
from .spot_dance import SpotDance
from .spot_docking import SpotDocking
from .spot_eap import SpotEAP
from .spot_graph_nav import SpotGraphNav
from .spot_images import SpotImages
from .spot_leash import SpotLeash, SpotLeashProtocol
from .spot_mission_wrapper import SpotMission
from .spot_world_objects import SpotWorldObjects
from .wrapper_helpers import RobotCommandData, RobotState

SPOT_CLIENT_NAME = "ros_spot"
MAX_COMMAND_DURATION = 1e5
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"


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
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.
    Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__("robot-state", client, logger, period_sec=1.0 / max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_state_async()
            callback_future.add_done_callback(lambda future: self._callback(future.result()))
            return callback_future


class AsyncMetrics(AsyncPeriodicQuery):
    """Class to get robot metrics at regular intervals.  get_robot_metrics_async query sent to the robot at every tick.
    Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncMetrics, self).__init__("robot-metrics", client, logger, period_sec=1.0 / max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_robot_metrics_async()
            callback_future.add_done_callback(lambda future: self._callback(future.result()))
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
                response = self._client.robot_command_feedback(self._spot_wrapper.last_stand_command)
                command_feedback = response.feedback.synchronized_feedback.mobility_command_feedback
                if command_feedback.status == basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    self._spot_wrapper.is_sitting = False
                    stand_status = command_feedback.stand_feedback.status
                    if stand_status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                        self._spot_wrapper.is_standing = True
                        self._spot_wrapper.last_stand_command = None
                    elif stand_status == basic_command_pb2.StandCommand.Feedback.STATUS_IN_PROGRESS:
                        self._spot_wrapper.is_standing = False
                    else:
                        self._logger.warning("Stand command in unknown state")
                        self._spot_wrapper.is_standing = False
                else:
                    self._logger.warning(
                        f"Stand command is not being processed anymore, current status: {command_feedback.status}"
                    )
                    self._spot_wrapper.last_stand_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper.last_stand_command = None

        if self._spot_wrapper.last_sit_command is not None:
            try:
                response = self._client.robot_command_feedback(self._spot_wrapper.last_sit_command)
                command_feedback = response.feedback.synchronized_feedback.mobility_command_feedback
                if command_feedback.status == basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    self._spot_wrapper.is_standing = False
                    sit_status = command_feedback.sit_feedback.status
                    if sit_status == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING:
                        self._spot_wrapper.is_sitting = True
                        self._spot_wrapper.last_sit_command = None
                    elif sit_status == basic_command_pb2.SitCommand.Feedback.STATUS_IN_PROGRESS:
                        self._spot_wrapper.is_sitting = False
                    else:
                        self._logger.warning("Sit command in unknown state")
                        self._spot_wrapper.is_sitting = False
                else:
                    self._logger.warning(
                        f"Sit command is not being processed anymore, current status: {command_feedback.status}"
                    )
                    self._spot_wrapper.last_sit_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper.last_sit_command = None

        is_moving = False

        if self._spot_wrapper.last_velocity_command_time is not None:
            if time.time() < self._spot_wrapper.last_velocity_command_time:
                is_moving = True
            else:
                self._spot_wrapper.last_velocity_command_time = None

        if self._spot_wrapper.last_trajectory_command is not None:
            try:
                response = self._client.robot_command_feedback(self._spot_wrapper.last_trajectory_command)
                status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
                )
                final_goal_status = (
                    response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.final_goal_status
                )

                if status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_STOPPED:
                    # Robot is stopped
                    self.stopped = True
                    self._spot_wrapper._trajectory_status_unknown = False
                    if (
                        final_goal_status
                        == basic_command_pb2.SE2TrajectoryCommand.Feedback.FINAL_GOAL_STATUS_ACHIEVABLE
                    ):
                        self._spot_wrapper.trajectory_complete = True
                        self._spot_wrapper.at_goal = True
                        self._spot_wrapper.last_trajectory_command = None
                        # Clear the command once at the goal
                    elif final_goal_status == basic_command_pb2.SE2TrajectoryCommand.Feedback.FINAL_GOAL_STATUS_BLOCKED:
                        self._spot_wrapper.trajectory_complete = True
                        self._spot_wrapper.last_trajectory_command = None
                    elif (
                        final_goal_status
                        == basic_command_pb2.SE2TrajectoryCommand.Feedback.FINAL_GOAL_STATUS_IN_PROGRESS
                    ):
                        self._logger.info(
                            "Robot stopped but trajectory still in progress. Perhaps something is in the way"
                        )
                    else:
                        self._logger.error("Robot stopped but final goal status is unknown.")
                        self._spot_wrapper.last_trajectory_command = None
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_STOPPING:
                    is_moving = True
                    self._spot_wrapper.is_stopping = True
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_IN_PROGRESS:
                    is_moving = True
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_UNKNOWN:
                    self._spot_wrapper.trajectory_status_unknown = True
                    self._spot_wrapper.last_trajectory_command = None
                else:
                    self._logger.error(
                        "Received trajectory command status outside of expected range, value is {}".format(status)
                    )
                    self._spot_wrapper.last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error("Error when getting robot command feedback: %s", e)
                self._spot_wrapper.last_trajectory_command = None

        self._spot_wrapper.is_moving = is_moving

        # Check if the robot is currently not receiving any velocity/trajectory commands as these will override any
        # previous sit/stand commands, and might leave the robot in an illogical state (sitting and moving at the
        # same time).
        if self._spot_wrapper.is_moving:
            self._spot_wrapper.is_standing = True
            self._spot_wrapper.is_sitting = False

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
        super(AsyncEStopMonitor, self).__init__("estop_alive", client, logger, period_sec=1.0 / rate)
        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if not self._spot_wrapper._estop_keepalive:
            self._logger.debug("No keepalive yet - the lease has not been claimed.")
            return

        last_estop_status = self._spot_wrapper._estop_keepalive.status_queue.queue[-1]
        if last_estop_status[0] == self._spot_wrapper._estop_keepalive.KeepAliveStatus.ERROR:
            self._logger.error("Estop keepalive has an error: {}".format(last_estop_status[1]))
        elif last_estop_status == self._spot_wrapper._estop_keepalive.KeepAliveStatus.DISABLED:
            self._logger.error("Estop keepalive is disabled: {}".format(last_estop_status[1]))
        else:
            # estop keepalive is ok
            pass


class SpotWrapper:
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""

    def __init__(
        self,
        username: str,
        password: str,
        hostname: str,
        port: int,
        robot_name: str,
        logger: logging.Logger,
        frame_prefix: typing.Optional[str] = None,
        start_estop: bool = True,
        estop_timeout: float = 9.0,
        rates: typing.Optional[typing.Dict[str, float]] = None,
        callbacks: typing.Optional[typing.Dict[str, typing.Callable]] = None,
        leash_interface: typing.Optional[SpotLeashProtocol] = None,
        use_take_lease: bool = False,
        get_lease_on_action: bool = False,
        continually_try_stand: bool = True,
        rgb_cameras: bool = True,
        payload_credentials_file: str = None,
        cert_resource_glob: typing.Optional[str] = None,
        gripperless: bool = False,
    ) -> None:
        """
        Args:
            username: Username for authentication with the robot
            password: Password for authentication with the robot
            hostname: ip address or hostname of the robot
            robot_name: Optional name of the robot
            frame_prefix: Optional prefix to be used for published message frames. If not provided, robot_name will
                          be used to construct the prefix instead.
            start_estop: If true, the wrapper will be an estop endpoint
            estop_timeout: Timeout for the estop in seconds. The SDK will check in with the wrapper at a rate of
                           estop_timeout/3 and if there is no communication the robot will execute a gentle stop.
            rates: Dictionary of rates to apply when retrieving various data from the robot
                   # TODO this should be an object to be unambiguous
            callbacks: Dictionary of callbacks which should be called when certain data is retrieved
                       # TODO this should be an object to be unambiguous
            leash_interface: Optional interface for lease and power management.
                             Defaults to SpotLeash if none is provided.
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
        self._continually_try_stand = continually_try_stand
        self._rgb_cameras = rgb_cameras
        self._frame_prefix = frame_prefix if frame_prefix is not None else (robot_name + "/" if robot_name else "")
        self._logger = logger
        self._estop_timeout = estop_timeout
        self._start_estop = start_estop
        self._keep_alive = True
        self._valid = True
        self.gripperless = gripperless

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._state = RobotState()
        self._trajectory_status_unknown = False
        self._command_data = RobotCommandData()

        try:
            self._sdk = create_standard_sdk(
                SPOT_CLIENT_NAME, service_clients=[MissionClient], cert_resource_glob=cert_resource_glob
            )
        except Exception as e:
            self._logger.error("Error creating SDK object: %s", e)
            self._valid = False
            return

        self._sdk.register_service_client(ChoreographyClient)
        self._logger.info("Initialising robot at {}".format(self._hostname))
        self._robot = self._sdk.create_robot(self._hostname)
        if port:
            self._robot.update_secure_channel_port(port)

        authenticated = False
        if self._payload_credentials_file:
            authenticated = self.authenticate_from_payload_credentials(
                self._robot, self._payload_credentials_file, self._logger
            )
        else:
            authenticated = self.authenticate(self._robot, self._username, self._password, self._logger)

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
                self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
                self._world_objects_client = self._robot.ensure_client(WorldObjectClient.default_service_name)
                self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
                self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
                self._map_processing_client = self._robot.ensure_client(MapProcessingServiceClient.default_service_name)
                self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
                self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
                self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
                self._docking_client = self._robot.ensure_client(DockingClient.default_service_name)
                self._spot_check_client = self._robot.ensure_client(SpotCheckClient.default_service_name)
                self._mission_client = self._robot.ensure_client(MissionClient.default_service_name)
                self._license_client = self._robot.ensure_client(LicenseClient.default_service_name)
                if not self.gripperless and self._robot.has_arm():
                    self._gripper_cam_param_client = self._robot.ensure_client(
                        GripperCameraParamClient.default_service_name
                    )
                else:
                    self._gripper_cam_param_client = None

                if self._license_client.get_feature_enabled([ChoreographyClient.license_name])[
                    ChoreographyClient.license_name
                ]:
                    self._is_licensed_for_choreography = True
                    self._choreography_client = self._robot.ensure_client(ChoreographyClient.default_service_name)
                else:
                    self._logger.info("Robot is not licensed for choreography")
                    self._is_licensed_for_choreography = False
                    self._choreography_client = None

                try:
                    self._point_cloud_client = self._robot.ensure_client(VELODYNE_SERVICE_NAME)
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
                    "finished booting yet. Will wait {} seconds and try again.".format(e, sleep_secs)
                )
                time.sleep(sleep_secs)

        if leash_interface is None:
            leash_interface = SpotLeash(
                robot=self._robot,
                logger=self._logger,
                always_take=use_take_lease,
                lease_on_action=get_lease_on_action,
                rate=max(0.0, self._rates.get("lease", 0.0)),
                callback=self._callbacks.get("lease", None),
            )
        self._spot_leash = leash_interface

        self._spot_leash_context = self._spot_leash.tie(self)
        self._spot_leash_context.bind(
            self,
            [
                self.stop,
                self.self_right,
                self.sit,
                self.simple_stand,
                self.stand,
                self.battery_change_pose,
                self.velocity_cmd,
                self.trajectory_cmd,
                self.execute_dance,
                self._robot_command,
                self._manipulation_request,
            ],
        )

        self._spot_leash_context.bind(
            self,
            [
                self.stop,
                self.power_on,
                self.safe_power_off,
                self.toggle_power,
            ],
            passive=True,
        )

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
        self._idle_task = AsyncIdle(self._robot_command_client, self._logger, 10.0, self)
        self._estop_monitor = AsyncEStopMonitor(self._estop_client, self._logger, 20.0, self)

        self._estop_endpoint = None
        self._estop_keepalive = None

        robot_tasks = [
            self._robot_state_task,
            self._robot_metrics_task,
            self._idle_task,
            self._estop_monitor,
        ]

        robot_tasks.extend(self._spot_leash.async_tasks)

        self._spot_check = SpotCheck(
            self._robot,
            self._logger,
            self._state,
            self._spot_check_client,
            self._robot_command_client,
        )

        self._spot_mission = SpotMission(
            self._robot,
            self._logger,
            self._state,
            self._mission_client,
            self._robot_command_client,
        )

        if self._robot.has_arm():
            self._spot_arm = SpotArm(
                self._robot,
                self._logger,
                self._state,
                self._robot_command_client,
                self._manipulation_api_client,
                self._robot_state_client,
                self._spot_leash_context,
                MAX_COMMAND_DURATION,
            )
        else:
            self._spot_arm = None

        self._spot_images = SpotImages(
            self._robot,
            self._logger,
            self._image_client,
            self._gripper_cam_param_client,
            self._rgb_cameras,
        )

        self._spot_docking = SpotDocking(
            self._robot,
            self._logger,
            self._state,
            self._command_data,
            self._docking_client,
            self._robot_command_client,
            self._spot_leash_context,
        )

        self._spot_graph_nav = SpotGraphNav(
            self._robot,
            self._logger,
            self._graph_nav_client,
            self._map_processing_client,
            self._robot_state_client,
            self._spot_leash_context,
        )

        if self._point_cloud_client:
            self._spot_eap = SpotEAP(
                self._logger,
                self._point_cloud_client,
                rate=self._rates.get("point_cloud", 0.0),
                callback=self._callbacks.get("lidar_points", None),
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
            self._spot_dance = SpotDance(self._robot, self._choreography_client, self._logger)

        self._robot_id = None

    @staticmethod
    def authenticate(robot: Robot, username: str, password: str, logger: logging.Logger) -> bool:
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
                logger.info("Trying to authenticate with robot from payload credentials...")
                robot.authenticate_from_payload_credentials(
                    *bosdyn.client.util.read_payload_credentials(payload_credentials_file)
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
    def spot_graph_nav(self) -> SpotGraphNav:
        """Return SpotGraphNav instance"""
        return self._spot_graph_nav

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
    def spot_mission(self) -> SpotMission:
        """Return SpotMission instance"""
        return self._spot_mission

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
        return self._spot_leash.resources

    @property
    def lease2(self) -> typing.Optional[Lease]:
        """Return the most recently `take`n or `acquire`d lease, or `None` if it is `release`d."""
        return self._spot_leash.lease

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
    def is_stopping(self) -> bool:
        return self._state.is_stopping

    @is_stopping.setter
    def is_stopping(self, state: bool) -> None:
        self._state.is_stopping = state

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
        if not self._valid:
            return False, "Wrapper is not valid"

        try:
            if not self._spot_leash.claim():
                return True, "We already claimed the lease"

            self._robot_id = self._robot.get_id()
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
        self._estop_endpoint = EstopEndpoint(self._estop_client, SPOT_CLIENT_NAME, self._estop_timeout)
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        if self._estop_keepalive:
            self._estop_keepalive.shutdown()
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe: bool = True) -> typing.Tuple[bool, str]:
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot
            on the ground first
        """
        try:
            if self._estop_keepalive is None:
                self._logger.warning("Software e-stop keepalive does not exist; creating one now and asserting e-stop.")
                self.resetEStop()

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
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def takeLease(self) -> typing.Tuple[bool, typing.Optional[Lease]]:
        """Take lease for the robot, which may have been taken already."""
        return self._spot_leash.grab(force=True)

    def getLease(self) -> typing.Tuple[bool, typing.Optional[Lease]]:
        """Get a lease for the robot and keep the lease alive automatically."""
        return self._spot_leash.grab()

    def yieldLease(self) -> None:
        self._spot_leash.yield_()

    def releaseLease(self) -> None:
        """Return the lease on the body."""
        self._spot_leash.release()

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
        self._valid = False
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.releaseLease()
        self.releaseEStop()

    def _robot_command(
        self,
        command_proto: robot_command_pb2.RobotCommand,
        end_time_secs: typing.Optional[float] = None,
        timesync_endpoint: typing.Optional[TimeSyncEndpoint] = None,
    ) -> typing.Tuple[bool, str, typing.Optional[int]]:
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

    def _manipulation_request(self, request_proto, end_time_secs=None, timesync_endpoint=None):
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

    def stop(self) -> typing.Tuple[bool, str]:
        """
        Stop any action the robot is currently doing.

        Returns:
            Tuple of bool success and a string message

        """
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self) -> typing.Tuple[bool, str]:
        """
        Have the robot self-right.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self) -> typing.Tuple[bool, str]:
        """
        Stop the robot's motion and sit down if able.

        Returns:
            Tuple of bool success and a string message

        """
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self.last_sit_command = response[2]
        return response[0], response[1]

    def sit_blocking(self) -> typing.Tuple[bool, str]:
        """
        Stop the robot's motion and sit down if able, and block until this function returns.

        Returns:
            Tuple of bool success and a string message

        """
        try:
            blocking_sit(command_client=self._robot_command_client, timeout_sec=10, update_frequency=1.0)
            return True, "Success"
        except Exception as e:
            self._logger.error(f"Unable to execute blocking sit: {e}")
            return False, str(e)

    def simple_stand(self, monitor_command: bool = True) -> typing.Tuple[bool, str]:
        """
        If the e-stop is enabled, and the motor power is enabled, stand the robot up.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
        if monitor_command:
            self.last_stand_command = response[2]
        return response[0], response[1]

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
                RobotCommandBuilder.synchro_stand_command(body_height=body_height, footprint_R_body=body_orientation)
            )
        else:
            # Otherwise just use the mobility params
            response = self._robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))

        if monitor_command:
            self.last_stand_command = response[2]
        return response[0], response[1]

    def battery_change_pose(self, dir_hint: int = 1) -> typing.Tuple[bool, str]:
        """
        Put the robot into the battery change pose

        Args:
            dir_hint: 1 rolls to the right side of the robot, 2 to the left

        Returns:
            Tuple of bool success and a string message
        """
        if self.is_sitting:
            response = self._robot_command(RobotCommandBuilder.battery_change_pose_command(dir_hint))
            return response[0], response[1]
        return False, "Call sit before trying to roll over"

    def safe_power_off(self) -> typing.Tuple[bool, str]:
        """
        Stop the robot's motion and sit if possible.  Once sitting, disable motor power.

        Returns:
            Tuple of bool success and a string message
        """
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def clear_behavior_fault(self, fault_id: int) -> typing.Tuple[bool, str, typing.Optional[bool]]:
        """
        Clear the behavior fault defined by the given id.

        Returns:
            Tuple of bool success, string message, and bool indicating whether the status was cleared
        """
        try:
            rid = self._robot_command_client.clear_behavior_fault(behavior_fault_id=fault_id, lease=None)
            return True, "Success", rid
        except Exception as e:
            return False, f"Exception while clearing behavior fault: {e}", None

    def power_on(self) -> typing.Tuple[bool, str]:
        """
        Enable the motor power if e-stop is enabled.

        Returns:
            Tuple of bool success and a string message
        """
        # Don't bother trying to power on if we are already powered on
        if not self.check_is_powered_on():
            # If we are requested to start the estop, we have to acquire it when powering on.
            # Ignore if estop is already acquired.
            if self._start_estop and self._estop_keepalive is None:
                self.resetEStop()
            try:
                self._logger.info("Powering on")
                self._robot.power_on()
            except Exception as e:
                return False, f"Exception while powering on: {e}"

            return True, "Success"

        return True, "Was already powered on"

    def set_mobility_params(self, mobility_params: spot_command_pb2.MobilityParams) -> None:
        """Set Params for mobility and movement

        Args:
            mobility_params: spot.MobilityParams, params for spot mobility commands.
        """
        self._mobility_params = mobility_params

    def get_mobility_params(self) -> spot_command_pb2.MobilityParams:
        """Get mobility params"""
        return self._mobility_params

    def velocity_cmd(
        self,
        v_x: float,
        v_y: float,
        v_rot: float,
        cmd_duration: float = 0.125,
        body_height: float = 0.0,
        use_obstacle_params: bool = False,
    ) -> typing.Tuple[bool, str]:
        """

        Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command
                          rate).
            body_height: Offset of the body relative to nominal stand height, in metres
            use_obstacle_params: (Optional) Flag to include the current obstacle_params. Default is False

        Returns:
            Tuple of bool success and a string message
        """
        end_time = time.time() + cmd_duration
        if body_height:
            current_mobility_params = self.get_mobility_params()
            height_adjusted_params = RobotCommandBuilder.mobility_params(
                body_height=body_height,
                locomotion_hint=current_mobility_params.locomotion_hint,
                stair_hint=current_mobility_params.stair_hint,
                external_force_params=current_mobility_params.external_force_params,
                stairs_mode=current_mobility_params.stairs_mode,
            )
            if use_obstacle_params:
                height_adjusted_params.obstacle_params.CopyFrom(current_mobility_params.obstacle_params)
            self.set_mobility_params(height_adjusted_params)
        response = self._robot_command(
            RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params),
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )
        self.last_velocity_command_time = end_time
        return response[0], response[1]

    def arm_joint_cmd(
        self, *, sh0: float, sh1: float, el0: float, el1: float, wr0: float, wr1: float
    ) -> typing.Tuple[bool, str]:
        """

        Send a single point arm joint command to the robot.

        Args:
            sh0: shoulder joint angle
            sh1: shoulder joint angle
            el0: elbow joint angle
            el1: elbow joint angle
            wr0: wrist joint angle
            wr1: wrist joint angle

        Returns:
            Tuple of bool success and a string message
        """
        robot_command = RobotCommandBuilder.arm_joint_command(sh0, sh1, el0, el1, wr0, wr1)
        response = self._robot_command(robot_command)
        return response[0], response[1]

    def trajectory_cmd(
        self,
        goal_x: float,
        goal_y: float,
        goal_heading: float,
        cmd_duration: float,
        frame_name: str = "odom",
        precise_position: bool = False,
        mobility_params: spot_command_pb2.MobilityParams = None,
        disable_vision_body_obstacle_avoidance: bool = False,
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
            disable_vision_body_obstacle_avoidance: flag to easily set mobility_params.obstacle_params flag above

        Returns:
            (bool, str) tuple indicating whether the command was successfully sent, and a message
        """
        if mobility_params is None:
            mobility_params = self._mobility_params
        if disable_vision_body_obstacle_avoidance:
            self._logger.warning("Caution: attempting to send mobility trajectory without obstacle avoidance")
            mobility_params.obstacle_params.disable_vision_body_obstacle_avoidance = (
                disable_vision_body_obstacle_avoidance
            )

        self._trajectory_status_unknown = False
        self.trajectory_complete = False
        self.stopped = False
        self.at_goal = False
        self.is_stopping = False
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
        self, robot_command: robot_command_pb2.RobotCommand, duration: float = MAX_COMMAND_DURATION
    ) -> typing.Tuple[bool, str, typing.Optional[int]]:
        end_time = time.time() + duration
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

    def get_robot_command_feedback(self, cmd_id: int) -> robot_command_pb2.RobotCommandFeedbackResponse:
        return self._robot_command_client.robot_command_feedback(cmd_id)

    def get_manipulation_command_feedback(self, cmd_id):
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(manipulation_cmd_id=cmd_id)

        return self._manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request
        )

    def toggle_power(self, should_power_on):
        """Power the robot on/off dependent on the current power state."""
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._robot_state_client.get_robot_state_async()
                state_response = future.result(timeout=10)  # 10 second timeout for waiting for the state response.
                if state_response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
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

    def stop_choreography(self) -> typing.Tuple[bool, str]:
        """
        Issue an empty choreography sequence to stop the robot from dancing
        """
        if self._is_licensed_for_choreography:
            return self._spot_dance.stop_choreography()
        else:
            return False, "Spot is not licensed for choreography"

    def execute_dance(self, data: typing.Union[ChoreographySequence, str], start_slice: int) -> typing.Tuple[bool, str]:
        """Upload and execute dance. Data can be passed as
        - ChoreographySequence: proto passed directly to function
        - str: file contents of a .csq read directly from disk
        """
        if self._is_licensed_for_choreography:
            return self._spot_dance.execute_dance(data, start_slice)
        else:
            return False, "Spot is not licensed for choreography"

    def execute_choreography_by_name(
        self, choreography_name: str, start_slice: int, use_async: bool = False
    ) -> typing.Tuple[bool, str]:
        """Execute choreography that has already been uploaded to the robot"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.execute_choreography_by_name(choreography_name, start_slice, use_async)
        else:
            return False, "Spot is not licensed for choreography"

    def upload_choreography(
        self, choreography_sequence: ChoreographySequence
    ) -> typing.Tuple[bool, str, UploadChoreographyResponse]:
        """Upload choreography sequence for later playback"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.upload_choreography(choreography_sequence)
        else:
            return False, "Spot is not licensed for choreography"

    def upload_animation(self, animation_name: str, animation_file_content: str) -> typing.Tuple[bool, str]:
        if self._is_licensed_for_choreography:
            return self._spot_dance.upload_animation(animation_name, animation_file_content)
        else:
            return False, "Spot is not licensed for choreography"

    def upload_animation_proto(self, animation_proto: Animation) -> typing.Tuple[bool, str]:
        if self._is_licensed_for_choreography:
            return self._spot_dance.upload_animation_proto(animation_proto)
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

    def get_choreography_status(
        self,
    ) -> typing.Tuple[bool, str, ChoreographyStatusResponse]:
        """get status of choreography playback"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.get_choreography_status()
        else:
            response = ChoreographyStatusResponse()
            response.status = ChoreographyStatusResponse.STATUS_UNKNOWN
            return (
                False,
                "Spot is not licensed for choreography",
                response,
            )

    def get_docking_state(self, **kwargs):
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state

    def start_recording_state(self, duration_seconds: float) -> typing.Tuple[bool, str, StartRecordingStateResponse]:
        """start recording robot motion as choreography"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.start_recording_state(duration_seconds)
        else:
            empty = StartRecordingStateResponse()
            return False, "Spot is not licensed for choreography", empty

    def stop_recording_state(
        self,
    ) -> typing.Tuple[bool, str, StopRecordingStateResponse]:
        """stop recording robot motion as choreography"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.stop_recording_state()
        else:
            empty = StopRecordingStateResponse()
            return False, "Spot is not licensed for choreography", empty

    def choreography_log_to_animation_file(
        self, name: str, fpath: str, has_arm: bool, **kwargs
    ) -> typing.Tuple[bool, str, str]:
        """save a choreography log to a file as an animation"""
        if self._is_licensed_for_choreography:
            return self._spot_dance.choreography_log_to_animation_file(name, fpath, has_arm, **kwargs)
        else:
            return False, "Spot is not licensed for choreography", ""

    def mutate_world_objects(
        self, mutate_request: world_object_pb2.MutateWorldObjectRequest
    ) -> world_object_pb2.MutateWorldObjectResponse:
        return self._spot_world_objects.mutate_world_objects(mutate_request)
