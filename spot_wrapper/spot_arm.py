import logging
import time
import typing

from bosdyn.api import arm_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api import manipulation_api_pb2
from bosdyn.api import robot_command_pb2
from bosdyn.api import synchronized_command_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.api import gripper_command_pb2
from bosdyn.client.robot import Robot
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    block_until_arm_arrives,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncEndpoint
from bosdyn.util import seconds_to_duration

from spot_wrapper.wrapper_helpers import RobotState, ClaimAndPowerDecorator


class SpotArm:
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_state: RobotState,
        robot_command_client: RobotCommandClient,
        manipulation_api_client: ManipulationApiClient,
        robot_state_client: RobotStateClient,
        max_command_duration: float,
        claim_and_power_decorator: ClaimAndPowerDecorator,
    ) -> None:
        """
        Constructor for SpotArm class.
        Args:
            robot: Robot object
            logger: Logger object
            robot_state: Object containing the robot's state as controlled by the wrapper
            robot_command_client: Command client to use to send commands to the robot
            manipulation_api_client: Command client to send manipulation commands to the robot
            robot_state_client: Client to retrieve state of the robot
            max_command_duration: Maximum duration for commands when using the manipulation command method
            claim_and_power_decorator: Object to use to decorate the functions on this object
        """
        self._robot = robot
        self._logger = logger
        self._robot_state = robot_state
        self._max_command_duration = max_command_duration
        self._robot_command_client = robot_command_client
        self._manipulation_api_client = manipulation_api_client
        self._robot_state_client = robot_state_client
        self._claim_and_power_decorator = claim_and_power_decorator
        self._claim_and_power_decorator.decorate_functions(
            self,
            decorated_funcs=[
                self.ensure_arm_power_and_stand,
                self.arm_stow,
                self.arm_unstow,
                self.arm_carry,
                self.arm_joint_move,
                self.force_trajectory,
                self.gripper_open,
                self.gripper_close,
                self.gripper_angle_open,
                self.hand_pose,
                self.grasp_3d,
            ],
        )

    def _manipulation_request(
        self,
        request_proto: manipulation_api_pb2,
        end_time_secs: typing.Optional[float] = None,
        timesync_endpoint: typing.Optional[TimeSyncEndpoint] = None,
    ):
        """Generic function for sending requests to the manipulation api of a robot.
        Args:
            request_proto: manipulation_api_pb2 object to send to the robot.
        """
        try:
            command_id = self._manipulation_api_client.manipulation_api_command(
                manipulation_api_request=request_proto,
                end_time_secs=end_time_secs,
                timesync_endpoint=timesync_endpoint,
            ).manipulation_cmd_id

            return True, "Success", command_id
        except Exception as e:
            self._logger.error(f"Unable to execute manipulation command: {e}")
            return False, str(e), None

    def manipulation_command(self, request: manipulation_api_pb2):
        end_time = time.time() + self._max_command_duration
        return self._manipulation_request(
            request,
            end_time_secs=end_time,
            timesync_endpoint=self._robot.time_sync.endpoint,
        )

    def get_manipulation_command_feedback(self, cmd_id):
        feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
            manipulation_cmd_id=cmd_id
        )

        return self._manipulation_api_client.manipulation_api_feedback_command(
            manipulation_api_feedback_request=feedback_request
        )

    def ensure_arm_power_and_stand(self) -> typing.Tuple[bool, str]:
        if not self._robot.has_arm():
            return False, "Spot with an arm is required for this service"

        try:
            self._logger.info("Spot is powering on within the timeout of 20 secs")
            self._robot.power_on(timeout_sec=20)
            assert self._robot.is_powered_on(), "Spot failed to power on"
            self._logger.info("Spot is powered on")
        except Exception as e:
            return (
                False,
                f"Exception occured while Spot or its arm were trying to power on: {e}",
            )

        if not self._robot_state.is_standing:
            blocking_stand(command_client=self._robot_command_client, timeout_sec=10)
            self._robot_state.is_standing = True
            self._logger.info("Spot is standing")
        else:
            self._logger.info("Spot is already standing")

        return True, "Spot has an arm, is powered on, and standing"

    def wait_for_arm_command_to_complete(self, cmd_id, timeout_sec=None):
        """
        Wait until a command issued to the arm complets. Wrapper around the SDK function for convenience

        Args:
            cmd_id: ID of the command that we are waiting on
            timeout_sec: After this time, timeout regardless of what the robot state is

        """
        block_until_arm_arrives(
            self._robot_command_client, cmd_id=cmd_id, timeout_sec=timeout_sec
        )

    def arm_stow(self) -> typing.Tuple[bool, str]:
        """
        Moves the arm to the stowed position

        Returns:
            Boolean success, string message
        """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Stow Arm
                stow = RobotCommandBuilder.arm_stow_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(stow)
                self._logger.info("Command stow issued")
                self.wait_for_arm_command_to_complete(cmd_id)

        except Exception as e:
            return False, f"Exception occured while trying to stow: {e}"

        return True, "Stow arm success"

    def arm_unstow(self) -> typing.Tuple[bool, str]:
        """
        Unstows the arm

        Returns:
            Boolean success, string message
        """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Unstow Arm
                unstow = RobotCommandBuilder.arm_ready_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(unstow)
                self._logger.info("Command unstow issued")
                self.wait_for_arm_command_to_complete(cmd_id)

        except Exception as e:
            return False, f"Exception occured while trying to unstow: {e}"

        return True, "Unstow arm success"

    def arm_carry(self) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Get Arm in carry mode
                carry = RobotCommandBuilder.arm_carry_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(carry)
                self._logger.info("Command carry issued")
                self.wait_for_arm_command_to_complete(cmd_id)

        except Exception as e:
            return False, f"Exception occured while carry mode was issued: {e}"

        return True, "Carry mode success"

    def make_arm_trajectory_command(
        self, arm_joint_trajectory: arm_command_pb2.ArmJointTrajectory
    ) -> robot_command_pb2.RobotCommand:
        """Helper function to create a RobotCommand from an ArmJointTrajectory.
        Copy from 'spot-sdk/python/examples/arm_joint_move/arm_joint_move.py'"""

        joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(
            trajectory=arm_joint_trajectory
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_joint_move_command=joint_move_command
        )
        sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        arm_sync_robot_cmd = robot_command_pb2.RobotCommand(
            synchronized_command=sync_arm
        )
        return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

    def arm_joint_move(self, joint_targets) -> typing.Tuple[bool, str]:
        # All perspectives are given when looking at the robot from behind after the unstow service is called
        # Joint1: 0.0 arm points to the front. positive: turn left, negative: turn right)
        # RANGE: -3.14 -> 3.14
        # Joint2: 0.0 arm points to the front. positive: move down, negative move up
        # RANGE: 0.4 -> -3.13 (
        # Joint3: 0.0 arm straight. moves the arm down
        # RANGE: 0.0 -> 3.1415
        # Joint4: 0.0 middle position. negative: moves ccw, positive moves cw
        # RANGE: -2.79253 -> 2.79253
        # # Joint5: 0.0 gripper points to the front. positive moves the gripper down
        # RANGE: -1.8326 -> 1.8326
        # Joint6: 0.0 Gripper is not rolled, positive is ccw
        # RANGE: -2.87 -> 2.87
        # Values after unstow are: [0.0, -0.9, 1.8, 0.0, -0.9, 0.0]
        if abs(joint_targets[0]) > 3.14:
            msg = "Joint 1 has to be between -3.14 and 3.14"
            self._logger.warning(msg)
            return False, msg
        elif joint_targets[1] > 0.4 or joint_targets[1] < -3.13:
            msg = "Joint 2 has to be between -3.13 and 0.4"
            self._logger.warning(msg)
            return False, msg
        elif joint_targets[2] > 3.14 or joint_targets[2] < 0.0:
            msg = "Joint 3 has to be between 0.0 and 3.14"
            self._logger.warning(msg)
            return False, msg
        elif abs(joint_targets[3]) > 2.79253:
            msg = "Joint 4 has to be between -2.79253 and 2.79253"
            self._logger.warning(msg)
            return False, msg
        elif abs(joint_targets[4]) > 1.8326:
            msg = "Joint 5 has to be between -1.8326 and 1.8326"
            self._logger.warning(msg)
            return False, msg
        elif abs(joint_targets[5]) > 2.87:
            msg = "Joint 6 has to be between -2.87 and 2.87"
            self._logger.warning(msg)
            return False, msg
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                trajectory_point = (
                    RobotCommandBuilder.create_arm_joint_trajectory_point(
                        joint_targets[0],
                        joint_targets[1],
                        joint_targets[2],
                        joint_targets[3],
                        joint_targets[4],
                        joint_targets[5],
                    )
                )
                arm_joint_trajectory = arm_command_pb2.ArmJointTrajectory(
                    points=[trajectory_point]
                )
                arm_command = self.make_arm_trajectory_command(arm_joint_trajectory)

                # Send the request
                cmd_id = self._robot_command_client.robot_command(arm_command)
                self.wait_for_arm_command_to_complete(cmd_id)
                return True, "Spot Arm moved successfully"

        except Exception as e:
            return False, f"Exception occured during arm movement: {e}"

    def create_wrench_from_forces_and_torques(self, forces, torques):
        force = geometry_pb2.Vec3(x=forces[0], y=forces[1], z=forces[2])
        torque = geometry_pb2.Vec3(x=torques[0], y=torques[1], z=torques[2])
        return geometry_pb2.Wrench(force=force, torque=torque)

    def force_trajectory(self, data) -> typing.Tuple[bool, str]:
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Duration in seconds.
                traj_duration = data.duration

                # first point on trajectory
                wrench0 = self.create_wrench_from_forces_and_torques(
                    data.forces_pt0, data.torques_pt0
                )
                t0 = seconds_to_duration(0)
                traj_point0 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench0, time_since_reference=t0
                )

                # Second point on the trajectory
                wrench1 = self.create_wrench_from_forces_and_torques(
                    data.forces_pt1, data.torques_pt1
                )
                t1 = seconds_to_duration(traj_duration)
                traj_point1 = trajectory_pb2.WrenchTrajectoryPoint(
                    wrench=wrench1, time_since_reference=t1
                )

                # Build the trajectory
                trajectory = trajectory_pb2.WrenchTrajectory(
                    points=[traj_point0, traj_point1]
                )

                # Build the trajectory request, putting all axes into force mode
                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=data.frame,
                    wrench_trajectory_in_task=trajectory,
                    x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                    rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                )
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command
                )
                synchronized_command = (
                    synchronized_command_pb2.SynchronizedCommand.Request(
                        arm_command=arm_command
                    )
                )
                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command
                )

                # Send the request
                cmd_id = self._robot_command_client.robot_command(robot_command)
                self._logger.info("Force trajectory command sent")
                self.wait_for_arm_command_to_complete(cmd_id)
        except Exception as e:
            return False, f"Exception occured during arm movement: {e}"

        return True, "Moved arm successfully"

    def gripper_open(self) -> typing.Tuple[bool, str]:
        """
        Fully opens the gripper

        Returns:
            Boolean success, string message
        """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Open gripper
                command = RobotCommandBuilder.claw_gripper_open_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open sent")
                self.block_until_gripper_command_completes(
                    self._robot_command_client, cmd_id
                )

        except Exception as e:
            return False, f"Exception occured while gripper was moving: {e}"

        return True, "Open gripper success"

    def gripper_close(self) -> typing.Tuple[bool, str]:
        """
        Closes the gripper

        Returns:
            Boolean success, string message
        """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # Close gripper
                command = RobotCommandBuilder.claw_gripper_close_command()

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper close sent")
                self.block_until_gripper_command_completes(
                    self._robot_command_client, cmd_id
                )

        except Exception as e:
            return False, f"Exception occured while gripper was moving: {e}"

        return True, "Closed gripper successfully"

    def gripper_angle_open(self, gripper_ang: float) -> typing.Tuple[bool, str]:
        """
        Takes an angle between 0 (closed) and 90 (fully opened) and opens the gripper at this angle

        Args:
            gripper_ang: Angle to which the gripper should be opened

        Returns:
            Boolean success, string message
        """
        if gripper_ang > 90 or gripper_ang < 0:
            return False, "Gripper angle must be between 0 and 90"
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                # The open angle command does not take degrees but the limits
                # defined in the urdf, that is why we have to interpolate
                closed = 0.349066
                opened = -1.396263
                angle = gripper_ang / 90.0 * (opened - closed) + closed
                command = RobotCommandBuilder.claw_gripper_open_angle_command(angle)

                # Command issue with RobotCommandClient
                cmd_id = self._robot_command_client.robot_command(command)
                self._logger.info("Command gripper open angle sent")
                self.block_until_gripper_command_completes(
                    self._robot_command_client, cmd_id
                )

        except Exception as e:
            return False, f"Exception occured while gripper was moving: {e}"

        return True, "Opened gripper successfully"

    def hand_pose(self, data) -> typing.Tuple[bool, str]:
        """
        Set the pose of the hand

        Args:
            data:

        Returns:
            Boolean success, string message
        """
        try:
            success, msg = self.ensure_arm_power_and_stand()
            if not success:
                self._logger.info(msg)
                return False, msg
            else:
                pose_point = data.pose_point
                # Move the arm to a spot in front of the robot given a pose for the gripper.
                # Build a position to move the arm to (in meters, relative to the body frame origin.)
                position = geometry_pb2.Vec3(
                    x=pose_point.position.x,
                    y=pose_point.position.y,
                    z=pose_point.position.z,
                )

                # # Rotation as a quaternion.
                rotation = geometry_pb2.Quaternion(
                    w=pose_point.orientation.w,
                    x=pose_point.orientation.x,
                    y=pose_point.orientation.y,
                    z=pose_point.orientation.z,
                )

                seconds = data.duration
                duration = seconds_to_duration(seconds)

                # Build the SE(3) pose of the desired hand position in the moving body frame.
                hand_pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
                hand_pose_traj_point = trajectory_pb2.SE3TrajectoryPoint(
                    pose=hand_pose, time_since_reference=duration
                )
                hand_trajectory = trajectory_pb2.SE3Trajectory(
                    points=[hand_pose_traj_point]
                )

                arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                    root_frame_name=data.frame,
                    pose_trajectory_in_task=hand_trajectory,
                    force_remain_near_current_joint_configuration=True,
                )
                arm_command = arm_command_pb2.ArmCommand.Request(
                    arm_cartesian_command=arm_cartesian_command
                )
                synchronized_command = (
                    synchronized_command_pb2.SynchronizedCommand.Request(
                        arm_command=arm_command
                    )
                )

                robot_command = robot_command_pb2.RobotCommand(
                    synchronized_command=synchronized_command
                )

                command = RobotCommandBuilder.build_synchro_command(robot_command)

                # Send the request
                cmd_id = self._robot_command_client.robot_command(robot_command)
                self._logger.info("Moving arm to position.")
                self.wait_for_arm_command_to_complete(cmd_id)

        except Exception as e:
            return (
                False,
                f"An error occured while trying to move arm \n Exception: {e}",
            )

        return True, "Moved arm successfully"

    @staticmethod
    def block_until_gripper_command_completes(
        robot_command_client: RobotCommandClient,
        cmd_id: int,
        timeout_sec: float = None,
    ) -> bool:
        """
        Helper that blocks until a gripper command achieves a finishing state

        Args:
         robot_command_client: Robot client, used to request feedback
         cmd_id: command ID returned by the robot when the grasp command was sent.
         timeout_sec: optional number of seconds after which we'll return no matter what the robot's state is.

        Returns:
            True if successfully completed the gripper command, False if it failed
        """
        if timeout_sec is not None:
            start_time = time.time()
            end_time = start_time + timeout_sec
            now = time.time()

        while timeout_sec is None or now < end_time:
            feedback_resp = robot_command_client.robot_command_feedback(cmd_id)
            gripper_state = (
                feedback_resp.feedback.gripper_command_feedback.claw_gripper_feedback.status
            )

            if gripper_state in [
                gripper_command_pb2.ClawGripperCommand.Feedback.STATUS_AT_GOAL,
                gripper_command_pb2.ClawGripperCommand.Feedback.STATUS_APPLYING_FORCE,
            ]:
                # If the gripper is commanded to close, it is successful either if it reaches the goal, or if it is
                # applying a force. Applying a force stops the command and puts it into force control mode.
                return True
            if (
                gripper_state
                == gripper_command_pb2.ClawGripperCommand.Feedback.STATUS_UNKNOWN
            ):
                return False

            time.sleep(0.1)
            now = time.time()
        return False

    @staticmethod
    def block_until_manipulation_completes(
        manipulation_client: ManipulationApiClient,
        cmd_id: int,
        timeout_sec: float = None,
    ) -> bool:
        """
        Helper that blocks until the arm achieves a finishing state for the specific manipulation command.

        Args:
         manipulation_client: manipulation client, used to request feedback
         cmd_id: command ID returned by the robot when the arm movement command was sent.
         timeout_sec: optional number of seconds after which we'll return no matter what
                      the robot's state is.

        Returns:
            True if successfully completed the manipulation, False if the manipulation failed
        """
        if timeout_sec is not None:
            start_time = time.time()
            end_time = start_time + timeout_sec
            now = time.time()

        while timeout_sec is None or now < end_time:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_id
            )

            # Send the request
            response = manipulation_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request
            )
            manipulation_state = response.current_state
            # TODO: Incorporate more of the feedback states if needed for different grasp commands.
            if manipulation_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                return True
            elif manipulation_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                return False

            time.sleep(0.1)
            now = time.time()
        return False

    def grasp_3d(
        self, frame: str, object_rt_frame: typing.List[float]
    ) -> typing.Tuple[bool, str]:
        """
        Attempt to grasp an object

        Args:
            frame: Frame in the which the object_rt_frame vector is given
            object_rt_frame: xyz position of the object in the given frame

        Returns:
            Bool indicating success, and a message with information.
        """
        try:
            frm = str(frame)
            pos = geometry_pb2.Vec3(
                x=object_rt_frame[0], y=object_rt_frame[1], z=object_rt_frame[2]
            )

            grasp = manipulation_api_pb2.PickObject(frame_name=frm, object_rt_frame=pos)

            # Ask the robot to pick up the object
            grasp_request = manipulation_api_pb2.ManipulationApiRequest(
                pick_object=grasp
            )
            # Send the request
            cmd_response = self._manipulation_api_client.manipulation_api_command(
                manipulation_api_request=grasp_request
            )

            success = self.block_until_manipulation_completes(
                self._manipulation_api_client, cmd_response.cmd_id
            )

            if success:
                msg = "Grasped successfully"
                self._logger.info(msg)
                return True, msg
            else:
                msg = "Grasp failed."
                self._logger.info(msg)
                return False, msg
        except Exception as e:
            return False, f"An error occured while trying to grasp from pose {e}"
