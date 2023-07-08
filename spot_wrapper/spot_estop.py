import logging
import typing
from dataclasses import dataclass

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.estop import (
    EstopClient,
    EstopEndpoint,
    EstopKeepAlive,
)
from bosdyn.client.robot import Robot


@dataclass
class KeepAliveHandle:
    """
    Helper class to store a handle to the keepalive which can be modified by the estop class and checked by the async
    monitor
    """

    keep_alive: typing.Optional[EstopKeepAlive] = None


class AsyncEStopMonitor(AsyncPeriodicQuery):
    """Class to check if the estop endpoint is still valid"""

    def __init__(
        self,
        client,
        logger: logging.Logger,
        rate: float,
        estop_keep_alive: KeepAliveHandle,
    ) -> None:
        """
        Attributes:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            estop_keep_alive: A handle to the estop keepalive object
        """
        super().__init__("estop_alive", client, logger, period_sec=1.0 / rate)
        self._estop_keep_alive = estop_keep_alive

    def _start_query(self) -> None:
        if not self._estop_keep_alive:
            self._logger.debug("No keepalive yet - the lease has not been claimed.")
            return

        last_estop_status = self._estop_keep_alive.keep_alive.status_queue.queue[-1]
        if last_estop_status[0] == EstopKeepAlive.KeepAliveStatus.ERROR:
            self._logger.error(f"Estop keepalive has an error: {last_estop_status[1]}")
        elif last_estop_status == EstopKeepAlive.KeepAliveStatus.DISABLED:
            self._logger.error(f"Estop keepalive is disabled: {last_estop_status[1]}")


class SpotEstop:
    """
    Module which handles the e-stop interaction and monitoring
    """

    def __init__(
        self,
        logger: logging.Logger,
        robot: Robot,
        estop_client: EstopClient,
        spot_client_name: str,
        estop_timeout: float = 9.0,
    ) -> None:
        """

        Args:
            logger:
            robot:
            estop_client:
            spot_client_name:
            estop_timeout:
        """
        self._logger = logger
        self._robot = robot
        self._estop_endpoint: typing.Optional[EstopEndpoint] = None
        self._keep_alive_handle = KeepAliveHandle()
        self._keep_alive: typing.Optional[EstopKeepAlive] = None
        self._estop_client = estop_client
        self._estop_timeout = estop_timeout
        self._spot_client_name = spot_client_name
        self._estop_task = AsyncEStopMonitor(
            self._estop_client, self._logger, 20.0, self._keep_alive_handle
        )

    def assert_estop(self, severe: bool = True) -> typing.Tuple[bool, str]:
        """
        Forces the robot into eStop state.

        Args:
            severe: If true, will cut motor power immediately.  If false, will try to settle the robot on the ground
                    first
        Returns:
            bool success and a message
        """
        try:
            if severe:
                self._keep_alive.stop()
            else:
                self._keep_alive.settle_then_cut()

            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to estop: {e}"

    def disengage_estop(self) -> typing.Tuple[bool, str]:
        """
        Disengages the E-Stop
        """
        try:
            self._keep_alive.allow()
            return True, "Success"
        except Exception as e:
            return False, f"Exception while attempting to disengage estop {e}"

    def release_estop(self) -> None:
        """
        Stop eStop keepalive
        """
        if self._keep_alive:
            self._keep_alive.stop()
            self._keep_alive = None
            self._keep_alive_handle.keep_alive = None
            self._estop_endpoint = None

    def reset_estop(self) -> None:
        """
        Get keepalive for eStop
        """
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, self._spot_client_name, self._estop_timeout
        )
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._keep_alive = EstopKeepAlive(self._estop_endpoint)
        self._keep_alive_handle.keep_alive = self._keep_alive

    @property
    def async_task(self) -> AsyncPeriodicQuery:
        """Returns the async PointCloudService task for the robot"""
        return self._estop_task
