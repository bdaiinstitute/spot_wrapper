import time

from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery, AsyncTasks
from bosdyn.geometry import EulerZXY

from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.power import PowerClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client import power

import bosdyn.api.robot_state_pb2 as robot_state_proto

front_image_sources = ['frontleft_fisheye_image', 'frontright_fisheye_image', 'frontleft_depth', 'frontright_depth']
"""List of image sources for front image periodic query"""
side_image_sources = ['left_fisheye_image', 'right_fisheye_image', 'left_depth', 'right_depth']
"""List of image sources for side image periodic query"""
rear_image_sources = ['back_fisheye_image', 'back_depth']
"""List of image sources for rear image periodic query"""

class AsyncRobotState(AsyncPeriodicQuery):
    """Class to get robot state at regular intervals.  get_robot_state_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback):
        super(AsyncRobotState, self).__init__("robot-state", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
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
        super(AsyncMetrics, self).__init__("robot-metrics", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
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
        super(AsyncLease, self).__init__("lease", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback

    def _start_query(self):
        callback_future = self._client.list_leases_async()
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
    """
    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__("robot_image_service", client, logger,
                                           period_sec=1.0/rate)
        self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        callback_future = self._client.get_image_async(self._image_requests)
        callback_future.add_done_callback(self._callback)
        return callback_future

class AsyncIdle(AsyncPeriodicQuery):
    """Class to check if the robot is moving, and if not, command a stand with the set mobility parameters

        Attributes:
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            spot_wrapper: A handle to the wrapper library
    """
    def __init__(self, logger, rate, spot_wrapper):
        super(AsyncIdle, self).__init__("lease", None, logger,
                                           period_sec=1.0/rate)

        self._spot_wrapper = spot_wrapper

    def _start_query(self):
        if time.time() - self._spot_wrapper.last_motion_command_time > 0.125 and self._spot_wrapper.is_standing:
            self._spot_wrapper.asyncStand()

class SpotWrapper():
    """Generic wrapper class to encompass all of of the V1 API features as well as maintaining leases automatically"""
    def __init__(self, username, password, token, hostname, logger, rates = {}, callbacks = {}):
        self._username = username
        self._password = password
        self._token = token
        self._hostname = hostname
        self._logger = logger
        self._rates = rates
        self._callbacks = callbacks
        self._keep_alive = True

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_standing = False
        self._last_motion_command_time = time.time()

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(build_image_request(source, image_format=image_pb2.Image.Format.FORMAT_RAW))

        self._sdk = create_standard_sdk('ros_spot')
        self._sdk.load_app_token(self._token)
        self._robot = self._sdk.create_robot(self._hostname)

        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._robot = None

        if self._robot:
            # Clients
            self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
            self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
            self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
            self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)

            # Async Tasks
            self._robot_state_task = AsyncRobotState(self._robot_state_client, self._logger, self._rates.get("robot_state", 1.0), self._callbacks.get("robot_state", lambda:None))
            self._robot_metrics_task = AsyncMetrics(self._robot_state_client, self._logger, self._rates.get("metrics", 1.0), self._callbacks.get("metrics", lambda:None))
            self._lease_task = AsyncLease(self._lease_client, self._logger, self._rates.get("lease", 1.0), self._callbacks.get("lease", lambda:None))
            self._front_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("front_image", 1.0), self._callbacks.get("front_image", lambda:None), self._front_image_requests)
            self._side_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("side_image", 1.0), self._callbacks.get("side_image", lambda:None), self._side_image_requests)
            self._rear_image_task = AsyncImageService(self._image_client, self._logger, self._rates.get("rear_image", 1.0), self._callbacks.get("rear_image", lambda:None), self._rear_image_requests)
            self._idle_task = AsyncIdle(self._logger, 10.0, self)

            self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)

            self._async_tasks = AsyncTasks(
                [self._robot_state_task, self._robot_metrics_task, self._lease_task, self._front_image_task, self._side_image_task, self._rear_image_task, self._idle_task])

            self._robot_id = None
            self._lease = None

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def metrics(self):
        """Return latest proto from the _robot_metrics_task"""
        return self._robot_metrics_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def is_standing(self):
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def last_motion_command_time(self):
        """Return time of last motion request time"""
        return self._last_motion_command_time

    def connect(self):
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            self._robot_id = self._robot.get_id()
            self._lease = self._lease_client.acquire()
            self.resetEStop()
        except (ResponseError, RpcError) as err:
            self._logger.error("Failed to initialize robot communication: %s", err)
            return False

    def updateTasks(self):
        """Loop through all periodic tasks and update their data if needed."""
        self._async_tasks.update()

    def resetEStop(self):
        """Get keepalive for eStop"""
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        if not self._estop_keepalive:
            self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True):
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_endpoint.stop()
            else:
                self._estop_endpoint.settle_then_cut()

            return True, "Success"
        except:
            return False, "Error"



    def releaseEStop(self):
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None

    def getLease(self):
        """Get a lease for the robot and keep the lease alive automatically."""
        return LeaseKeepAlive(self._lease_client)

    def disconnect(self):
        """Release control of robot as gracefully as posssible."""
        self._logger.info("Shutting down ROS interface")
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        releaseEStop()
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def _robot_command(self, desc, command_proto, end_time_secs=None):
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
        """
        try:
            id = self._robot_command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)
            return True, "Success"
        except Exception as e:
            return False, str(type(e))

    def _async_robot_command(self, desc, command_proto, end_time_secs=None):
        """Generic non-blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 object to send to the robot.  Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
        """
        return self._robot_command_client.robot_command_async(lease=None, command=command_proto, end_time_secs=end_time_secs)

    def stop(self):
        """Stop the robot's motion."""
        return self._robot_command('stop', RobotCommandBuilder.stop_command())

    def self_right(self):
        """Have the robot self-right itself."""
        return self._robot_command('self-right', RobotCommandBuilder.selfright_command())

    def sit(self):
        """Stop the robot's motion and sit down if able."""
        self._is_standing = False
        return self._robot_command('sit', RobotCommandBuilder.sit_command())

    def asyncStand(self):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up.  Async version.  Doesn't set _is_standing"""
        return self._async_robot_command('stand', RobotCommandBuilder.stand_command(params=self._mobility_params))

    def stand(self):
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        self._is_standing = True
        return self._robot_command('stand', RobotCommandBuilder.stand_command(params=self._mobility_params))

    def safe_power_off(self):
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        return self._robot_command('safe-power-off', RobotCommandBuilder.safe_power_off_command())

    def power_on(self):
        """Enble the motor power if e-stop is enabled."""
        try:
            power.power_on(self._power_client)
            return True, "Success"
        except:
            return False, "Error"

    def set_mobility_params(self, body_height=0, footprint_R_body=EulerZXY(), locomotion_hint=1, stair_hint=False, external_force_params=None):
        """Define body, locomotion, and stair parameters.

        Args:
            body_height: Body height in meters
            footprint_R_body: (EulerZXY) – The orientation of the body frame with respect to the footprint frame (gravity aligned framed with yaw computed from the stance feet)
            locomotion_hint: Locomotion hint
            stair_hint: Boolean to define stair motion
        """
        self._mobility_params = RobotCommandBuilder.mobility_params(body_height, footprint_R_body, locomotion_hint, stair_hint, external_force_params)

    def velocity_cmd(self, v_x, v_y, v_rot, cmd_duration=0.125):
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters
            v_y: Velocity in the Y direction in meters
            v_rot: Angular velocity around the Z axis in radians
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 125ms (assuming 10Hz command rate).
        """
        self.last_motion_command = time.time()
        return self._async_robot_command("ros_cmd_vel",
                                  RobotCommandBuilder.velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params),
                                  end_time_secs=time.time() + cmd_duration)