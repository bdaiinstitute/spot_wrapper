import logging
from typing import Optional

from bosdyn.api.mission import nodes_pb2
from bosdyn.client import RpcError, robot_command
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
    ) -> None:
        self._robot = robot
        self._logger = logger
        self._mission_client: MissionClient = mission_client
        self._robot_command_client = robot_command_client
        self._robot_state = robot_state

    def load_mission(self, root: nodes_pb2.Node, leases=None, data_chunk_byte_size: Optional[int] = None):
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
        if leases is None:
            leases = []
        if data_chunk_byte_size:
            return self._load_mission_as_chunks(root, leases, data_chunk_byte_size)
        try:
            return self._mission_client.load_mission_async(root, leases)
        except RpcError:
            return False, "Could not communicate with the robot"
        except CompilationError as e:
            return False, f"The mission failed to compile: {e}"
        except ValidationError as e:
            return False, f"The mission could not be validated: {e}"

    def _load_mission_as_chunks(self, root: nodes_pb2.Node, leases=None, data_chunk_byte_size: int = 1000 * 1000):
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
        if leases is None:
            leases = []
        try:
            return self._mission_client.load_mission_as_chunks2(root, leases, data_chunk_byte_size)
        except RpcError:
            return False, "Could not communicate with the robot"
        except CompilationError as e:
            return False, f"The mission failed to compile: {e}"
        except ValidationError as e:
            return False, f"The mission could not be validated: {e}"

    def get_mission_info(self):
        """Get static information about the loaded mission.

        Raises:
            RpcError: Problem communicating with the robot.
        """
        try:
            return self._mission_client.get_info()
        except RpcError:
            return False, "Could not communicate with the robot"

    def play_mission(
        self,
        pause_time_secs: int,
        leases=None,
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
        if leases is None:
            leases = []
        try:
            return self._mission_client.play_mission_async(pause_time_secs, leases, settings)
        except RpcError:
            return False, "Could not communicate with the robot"
        except NoMissionError:
            return False, "No mission loaded"

    def get_mission_state(
        self,
        upper_tick_bound: Optional[int] = None,
        lower_tick_bound: Optional[int] = None,
        past_ticks: Optional[int] = None,
    ):
        """Get the state of the current playing mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            return self._mission_client.get_state_async(upper_tick_bound, lower_tick_bound, past_ticks)
        except RpcError:
            return False, "Could not communicate with the robot"
        except NoMissionPlayingError:
            return False, "No mission playing"

    def pause_mission(self):
        """Pause the current mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            return self._mission_client.pause_mission_async()
        except RpcError:
            return False, "Could not communicate with the robot"
        except NoMissionPlayingError:
            return False, "No mission playing"

    def restart_mission(self, pause_time_secs, leases=None, settings=None):
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
        if leases is None:
            leases = []
        try:
            return self._mission_client.restart_mission_async(pause_time_secs, leases, settings)
        except RpcError:
            return False, "Could not communicate with the robot"
        except NoMissionError:
            return False, "No mission loaded"
        except ValidationError as e:
            return False, f"The mission could not be validated: {e}"

    def stop_mission(self):
        """Stop the current mission
        Raises:
            RpcError: Problem communicating with the robot.
            NoMissionPlayingError: No mission playing.
        """
        try:
            return self._mission_client.stop_mission_async()
        except RpcError:
            return False, "Could not communicate with the robot"
        except NoMissionPlayingError:
            return False, "No mission playing"
