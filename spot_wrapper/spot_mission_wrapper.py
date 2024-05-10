import logging
from typing import Optional

from bosdyn.api.mission import nodes_pb2
from bosdyn.client import RpcError, robot_command
from bosdyn.client.lease import Lease, LeaseClient, LeaseWallet
from bosdyn.client.robot import Robot
from bosdyn.mission.client import (
    CompilationError,
    MissionClient,
    NoMissionError,
    NoMissionPlayingError,
    ValidationError,
)

from spot_wrapper.wrapper_helpers import RobotState


class SpotMission:
    """
    Allow access to mission functionality through the SDK
    """

    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_state: RobotState,
        mission_client: MissionClient,
        robot_command_client: robot_command.RobotCommandClient,
        lease_client: LeaseClient,
    ) -> None:
        self._robot = robot
        self._logger = logger
        self._mission_client: MissionClient = mission_client
        self._robot_command_client = robot_command_client
        self._lease_client = lease_client
        self._robot_state = robot_state
        self._spot_check_resp = None
        self._lease = None
        self._lease_wallet: LeaseWallet = self._lease_client.lease_wallet

    def _get_lease(self) -> Lease:
        self._lease = self._lease_wallet.get_lease()
        return self._lease

    def _load_mission(self, root: nodes_pb2.Node, leases: list[Lease] = [], data_chunk_byte_size: Optional[int] = None):
        """Load a mission
        Args:
            root: Root node in a mission.
            leases: All leases necessary to initialize a mission.
            data_chunk_byte_size: Optional max size of each streamed message
        Raises:
            RpcError: Problem communicating with the robot.
            bosdyn.mission.client.CompilationError: The mission failed to compile.
            bosdyn.mission.client.ValidationError: The mission failed to validate.
        """
        if data_chunk_byte_size:
            return self._load_mission_as_chunks(root, leases, data_chunk_byte_size)
        try:
            resp = self._mission_client.load_mission_async(root, leases)
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except CompilationError as e:
            resp = (False, f"The mission failed to compile: {e}")
        except ValidationError as e:
            resp = (False, f"The mission could not be validated: {e}")
        return resp

    def _load_mission_as_chunks(
        self, root: nodes_pb2.Node, leases: list[Lease] = [], data_chunk_byte_size: int = 1000 * 1000
    ):
        """Load a mission onto the robot.
        Args:
            root: Root node in a mission.
            leases: All leases necessary to initialize a mission.
            data_chunk_byte_size: max size of each streamed message
        Raises:
            RpcError: Problem communicating with the robot.
            bosdyn.mission.client.CompilationError: The mission failed to compile.
            bosdyn.mission.client.ValidationError: The mission failed to validate.
        """
        try:
            resp = self._mission_client.load_mission_as_chunks2(root, leases, data_chunk_byte_size)
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except CompilationError as e:
            resp = (False, f"The mission failed to compile: {e}")
        except ValidationError as e:
            resp = (False, f"The mission could not be validated: {e}")
        return resp

    def get_mission_info(self):
        """Get static information about the loaded mission.

        Raises:
            RpcError: Problem communicating with the robot.
        """
        try:
            return self._mission_client.get_info()
        except RpcError:
            return False, "Could not communicate with the robot"

    def _play_mission(
        self,
        pause_time_secs: int,
        leases: list[Lease] = [],
        settings=None,
    ):
        """Play loaded mission or continue a paused mission
        Args:
          pause_time_secs: Absolute time when the mission should pause execution. Subsequent RPCs
              will override this value, so you can use this to say "if you don't hear from me again,
              stop running the mission at this time."
          leases: Leases the mission service will need to use. Unlike other clients, these MUST
              be specified.
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionError: No mission Loaded.
        """
        try:
            resp = self._mission_client.play_mission_async(pause_time_secs, leases, settings)
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except NoMissionError:
            resp = (False, "No mission loaded")
        return resp

    def _get_mission_state(self, upper_tick_bound: int = None, lower_tick_bound: int = None, past_ticks: int = None):
        """Get the state of the current playing mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            resp = self._mission_client.get_state_async(upper_tick_bound, lower_tick_bound, past_ticks)
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except NoMissionPlayingError:
            resp = (False, "No mission playing")
        return resp

    def _pause_mission(self):
        """Pause the current mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            resp = self._mission_client.pause_mission_async()
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except NoMissionPlayingError:
            resp = (False, "No mission playing")
        return resp

    def _restart_mission(self, pause_time_secs, leases: list[Lease] = [], settings=None):
        """Restart mission from the beginning
        Args:
          pause_time_secs: Absolute time when the mission should pause execution. Subsequent RPCs
              to RestartMission will override this value, so you can use this to say "if you don't hear
              from me again, stop running the mission at this time."
          leases: Leases the mission service will need to use. Unlike other clients, these MUST
              be specified.
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionError: No Mission Loaded.
            bosdyn.mission.client.ValidationError: The mission failed to validate.
        """
        try:
            resp = self._mission_client.restart_mission_async(pause_time_secs, leases, settings)
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except NoMissionError:
            resp = (False, "No mission loaded")
        except ValidationError as e:
            resp = (False, f"The mission could not be validated: {e}")
        return resp

    def _stop_mission(self):
        """Stop the current mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            resp = self._mission_client.stop_mission_async()
        except RpcError:
            resp = (False, "Could not communicate with the robot")
        except NoMissionPlayingError:
            resp = (False, "No mission playing")
        return resp
