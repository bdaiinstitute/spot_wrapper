import logging
import time
import typing

from bosdyn.api import header_pb2
from bosdyn.client import robot_command
from bosdyn.client.lease import LeaseClient, LeaseWallet, Lease
from bosdyn.client.robot import Robot
from bosdyn.client.spot_check import SpotCheckClient, run_spot_check
from bosdyn.client.spot_check import spot_check_pb2
from google.protobuf.timestamp_pb2 import Timestamp
from spot_wrapper.wrapper_helpers import RobotState


class SpotCheck:
    """
    Allow access to spot check functionality through the SDK
    """

    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_state: RobotState,
        spot_check_client: SpotCheckClient,
        robot_command_client: robot_command.RobotCommandClient,
        lease_client: LeaseClient,
    ) -> None:
        self._robot = robot
        self._logger = logger
        self._spot_check_client: SpotCheckClient = spot_check_client
        self._robot_command_client = robot_command_client
        self._lease_client = lease_client
        self._robot_state = robot_state
        self._spot_check_resp = None
        self._lease = None
        self._lease_wallet: LeaseWallet = self._lease_client.lease_wallet

    @property
    def spot_check_resp(self) -> spot_check_pb2.SpotCheckFeedbackResponse:
        return self._spot_check_resp

    def _get_lease(self) -> Lease:
        self._lease = self._lease_wallet.get_lease()
        return self._lease

    def _feedback_error_check(
        self, resp: spot_check_pb2.SpotCheckFeedbackResponse
    ) -> typing.Tuple[bool, str]:
        """Check for errors in the feedback response"""

        # Save results from Spot Check
        self._spot_check_resp = resp

        errorcode_mapping = {
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_UNEXPECTED_POWER_CHANGE: "Unexpected power change",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_INIT_IMU_CHECK: "Robot body is not flat on the ground",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_INIT_NOT_SITTING: "Robot body is not close to a sitting pose",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_LOADCELL_TIMEOUT: "Timeout during loadcell calibration",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_POWER_ON_FAILURE: "Error enabling motor power",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_ENDSTOP_TIMEOUT: "Timeout during endstop calibration",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_FAILED_STAND: "Robot failed to stand",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_CAMERA_TIMEOUT: "Timeout during camera check",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_GROUND_CHECK: "Flat ground check failed",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_POWER_OFF_FAILURE: "Robot failed to power off",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_REVERT_FAILURE: "Robot failed to revert calibration",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_FGKC_FAILURE: "Robot failed to do flat ground kinematic calibration",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_GRIPPER_CAL_TIMEOUT: "Timeout during gripper calibration",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_ARM_CHECK_COLLISION: "Arm motion would cause collisions",
            spot_check_pb2.SpotCheckFeedbackResponse.ERROR_ARM_CHECK_TIMEOUT: "Timeout during arm joint check",
        }
        # Check for common errors
        if resp.header.error.code in (2, 3):
            return False, str(resp.header.error.message)
        if resp.error > 1:
            return False, errorcode_mapping[resp.error]

        return True, "Successfully ran Spot Check"

    def _req_feedback(self) -> spot_check_pb2.SpotCheckFeedbackResponse:
        """
        Get feedback from the spot check command

        Returns:
            Feedback from the spot check command
        """
        start_time_seconds, start_time_ns = int(time.time()), int(time.time_ns() % 1e9)
        req = spot_check_pb2.SpotCheckFeedbackRequest(
            header=header_pb2.RequestHeader(
                request_timestamp=Timestamp(
                    seconds=start_time_seconds, nanos=start_time_ns
                ),
                client_name="spot-check",
                disable_rpc_logging=False,
            )
        )
        resp: spot_check_pb2.SpotCheckFeedbackResponse = (
            self._spot_check_client.spot_check_feedback(req)
        )

        self._spot_check_resp = resp

        return resp

    def _spot_check_cmd(self, command: spot_check_pb2.SpotCheckCommandRequest):
        """Send a Spot Check command"""
        start_time_seconds, start_time_ns = int(time.time()), int(time.time_ns() % 1e9)
        req = spot_check_pb2.SpotCheckCommandRequest(
            header=header_pb2.RequestHeader(
                request_timestamp=Timestamp(
                    seconds=start_time_seconds, nanos=start_time_ns
                ),
                client_name="spot-check",
                disable_rpc_logging=False,
            ),
            lease=self._get_lease().lease_proto,
            command=command,
        )
        self._spot_check_client.spot_check_command(req)

    def stop_check(self) -> typing.Tuple[bool, str]:
        """Stop the Spot Check
        Note: This may cause the robot to enter a FaultState. Use only in emergencies.
        """
        self._spot_check_cmd(spot_check_pb2.SpotCheckCommandRequest.COMMAND_ABORT)

        # Get feedback
        resp = self._req_feedback()

        # Check for errors
        success, status = self._feedback_error_check(resp)

        if success:
            status = "Successfully stopped Spot Check"
            self._logger.info(status)
        else:
            self._logger.error("Failed to stop Spot Check")

        return success, status

    def revert_calibration(self) -> typing.Tuple[bool, str]:
        """Revert calibration for Spot Check"""
        self._spot_check_cmd(spot_check_pb2.SpotCheckCommandRequest.COMMAND_REVERT_CAL)

        # Get feedback
        resp = self._req_feedback()

        # Check for errors
        success, status = self._feedback_error_check(resp)

        if success:
            status = "Successfully reverted calibration"
            self._logger.info(status)
        else:
            status = "Failed to revert calibration"
            self._logger.error(status)

        return success, status

    def start_check(self) -> typing.Tuple[bool, str]:
        """Start the Spot Check"""
        # Make sure we're powered on and sitting
        try:
            self._robot.power_on()
            if not self._robot_state.is_sitting:
                robot_command.blocking_sit(
                    command_client=self._robot_command_client, timeout_sec=10
                )
                self._logger.info("Spot is sitting")
            else:
                self._logger.info("Spot is already sitting")

            self._spot_check_cmd(spot_check_pb2.SpotCheckCommandRequest.COMMAND_START)

            # Get feedback
            resp = self._req_feedback()

            # Check for errors
            success, status = self._feedback_error_check(resp)

            if success:
                status = "Successfully started Spot Check"
                self._logger.info(status)
            else:
                self._logger.error("Failed to start Spot Check")
            return success, status

        except Exception as e:
            return False, f"Exception running start_check from spot check: {str(e)}"

    def blocking_check(
        self,
        timeout_sec: int = 360,
        update_freq: float = 0.25,
        verbose: bool = False,
    ) -> typing.Tuple[bool, str]:
        """Check the robot
        Args:
            timeout_sec: Timeout for the blocking check
            update_freq: Frequency to update the check
            verbose: Whether to print the check status
        Returns:
            Tuple of (success, message)
        """
        try:
            # Make sure we're powered on and sitting
            self._robot.power_on()
            if not self._robot_state.is_sitting:
                robot_command.blocking_sit(
                    command_client=self._robot_command_client, timeout_sec=10
                )
                self._logger.info("Spot is sitting")
            else:
                self._logger.info("Spot is already sitting")

            # Check the robot and block for timeout_sec
            self._logger.info("Blocking Spot Check is starting!")
            resp: spot_check_pb2.SpotCheckFeedbackResponse = run_spot_check(
                self._spot_check_client,
                self._get_lease(),
                timeout_sec,
                update_freq,
                verbose,
            )

            self._logger.info("Blocking Spot Check ran successfully!")
            success, status = self._feedback_error_check(resp)

            return success, status

        except Exception as e:
            self._logger.error(
                "Exception thrown during blocking spot check: {}".format(e)
            )
            return False, str(e)

    def get_feedback(self) -> spot_check_pb2.SpotCheckFeedbackResponse:
        """Get feedback from Spot Check"""
        resp = self._req_feedback()
        return resp[0], "Got only feedback from Spot Check"
