import logging
import typing

from bosdyn.api.docking import docking_pb2
from bosdyn.client import robot_command
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.client.robot import Robot

from spot_wrapper.wrapper_helpers import (
    RobotState,
    RobotCommandData,
    ClaimAndPowerDecorator,
)


class SpotDocking:
    """
    Interactions with spot's autonomous docking station
    """

    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_state: RobotState,
        command_data: RobotCommandData,
        docking_client: DockingClient,
        robot_command_client: robot_command.RobotCommandClient,
        claim_and_power_decorator: ClaimAndPowerDecorator,
    ) -> None:
        self._robot = robot
        self._logger = logger
        self._command_data = command_data
        self._docking_client: DockingClient = docking_client
        self._robot_command_client = robot_command_client
        self._robot_state = robot_state
        self._claim_and_power_decorator = claim_and_power_decorator
        # Decorate the functions so that they take the lease. Dock function needs to power on because it might have
        # to move the robot, the undock
        self._claim_and_power_decorator.decorate_functions(
            self, decorated_funcs=[self.dock, self.undock]
        )

    def dock(self, dock_id: int) -> typing.Tuple[bool, str]:
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            if not self._robot_state.is_standing:
                robot_command.blocking_stand(
                    command_client=self._robot_command_client, timeout_sec=10
                )
                self._logger.info("Spot is standing")
            else:
                self._logger.info("Spot is already standing")
            # Dock the robot
            self._command_data.last_docking_command = dock_id
            blocking_dock_robot(self._robot, dock_id)
            self._command_data.last_docking_command = None
            return True, "Success"
        except Exception as e:
            return False, f"Exception while trying to dock: {e}"

    def undock(self, timeout: int = 20) -> typing.Tuple[bool, str]:
        """Power motors on and undock the robot from the station."""
        try:
            # Maker sure we're powered on
            self._robot.power_on()
            # Undock the robot
            blocking_undock(self._robot, timeout)
            return True, "Success"
        except Exception as e:
            return False, f"Exception while trying to undock: {e}"

    def get_docking_state(self, **kwargs) -> docking_pb2.DockState:
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state
