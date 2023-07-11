"""Helper classes for the wrapper. This file is necessary to prevent circular imports caused by the modules also
using these classes"""
import typing
from dataclasses import dataclass


@dataclass()
class RobotState:
    """
    Dataclass which stores information about the robot's state. The values in it may be changed by methods
    """

    is_sitting: bool = True
    is_standing: bool = False
    is_moving: bool = False
    at_goal: bool = False
    near_goal: bool = False


@dataclass()
class RobotCommandData:
    """
    Store data about the commands the wrapper sends to the SDK. Running a command returns an integer value
    representing that command's ID. These values are used to monitor the progress of the command and modify attributes
    of RobotState accordingly. The values should be reset to none when the command completes.
    """

    last_stand_command: typing.Optional[int] = None
    last_sit_command: typing.Optional[int] = None
    last_docking_command: typing.Optional[int] = None
    last_trajectory_command: typing.Optional[int] = None
    # Was the last trajectory command requested to be precise
    last_trajectory_command_precise: typing.Optional[bool] = None
    last_velocity_command_time: typing.Optional[float] = None
