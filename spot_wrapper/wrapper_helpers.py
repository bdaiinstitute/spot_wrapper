"""Helper classes for the wrapper. This file is necessary to prevent circular imports caused by the modules also
using these classes"""
import typing
import functools
from dataclasses import dataclass


class ClaimAndPowerDecorator:
    """
    Some functions in the wrapper require the lease to be claimed and the robot to be powered on before they can
    function.

    This class provides a portable way of wrapping the functions of the wrapper or modules to enable them to do
    that. It can be passed around to modules which can then decorate their functions with it, allowing them to claim
    and power on as needed.

    Note that this decorator is not intended to be applied using the @ syntax. It should be applied during or after
    object instantiation.
    """

    def __init__(
        self,
        power_on_function: typing.Callable[[], typing.Tuple[bool, str]],
        claim_function: typing.Callable[[], typing.Tuple[bool, str]],
        get_lease_on_action: bool = False,
    ) -> None:
        self.power_on = power_on_function
        self.claim = claim_function
        self._get_lease_on_action = get_lease_on_action

    def _make_function_take_lease_and_power_on(
        self, func: typing.Callable, power_on: bool = True
    ) -> typing.Callable:
        """
        Decorator which tries to acquire the lease before executing the wrapped function

        Args:
            func: Function that is being wrapped
            power_on: If true, power on after claiming the lease. For the vast majority of cases this is needed

        Returns:
            Decorator which will wrap the decorated function
        """

        @functools.wraps(func)
        def wrapper_try_claim(*args, **kwargs) -> typing.Callable:
            # Note that because we are assuming that this decorator is used only on instantiated classes,
            # this function does not take a self arg. The self arg is necessary when using the @ syntax because at
            # that point the class has not yet been instantiated. In this case, the func we receive is already a bound
            # method, as opposed to an unbound one. A bound function has the "self" instance built in.
            if self._get_lease_on_action:
                # Ignore success or failure of these functions. If they fail, then the function that is being wrapped
                # will fail and the caller will be able to handle from there.
                self.claim()
                if power_on:
                    self.power_on()
            return func(*args, **kwargs)

        return wrapper_try_claim

    def make_function_take_lease_and_power_on(
        self, decorated_object, function: typing.Callable, power_on: bool = True
    ) -> None:
        """
        Decorate a function of an object with this class's decorator. After being decorated, when the function is
        called, it will forcefully take the lease, then power on the robot if that option is specified.

        Args:
            decorated_object: The object whose function is to be decorated
            function: The function to be decorated
            power_on: If true, power on the robot after claiming

        Raises:
            AttributeError: if the object passed does not have a function with the same name as the given function
        """
        function_name = function.__name__
        if not hasattr(decorated_object, function_name):
            raise AttributeError(
                f"Requested decoration of function {function_name} of object {decorated_object}, but the object does "
                f"not have a function by that name."
            )

        setattr(
            decorated_object,
            function_name,
            self._make_function_take_lease_and_power_on(function, power_on=power_on),
        )

    def decorate_functions(
        self,
        decorated_object,
        decorated_funcs: typing.List[typing.Callable],
        decorated_funcs_no_power: typing.Optional[typing.List[typing.Callable]] = None,
    ) -> None:
        """
        Decorate the specified functions of the given object with this class's decorator.

        Args:
            decorated_object: Object which contains the functions to be decorated
            decorated_funcs: List of the functions of the object which should be decorated. When called,
                             these functions will forcefully take the lease and power on the robot
            decorated_funcs_no_power: Same as decorated_funcs, but will calling these will not power on the robot.
        """

        for func in decorated_funcs:
            self.make_function_take_lease_and_power_on(decorated_object, func)

        if decorated_funcs_no_power is None:
            decorated_funcs_no_power = []

        for func in decorated_funcs_no_power:
            self.make_function_take_lease_and_power_on(
                decorated_object, func, power_on=False
            )


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
