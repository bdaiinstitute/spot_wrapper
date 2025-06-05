import logging
from collections.abc import Sequence
from typing import Any, Callable, List, Optional, Protocol, Tuple

from bosdyn.api import lease_pb2
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Lease, LeaseClient, LeaseKeepAlive
from bosdyn.client.robot import Robot

from .wrapper_helpers import ClaimAndPowerDecorator


class SpotLeashContextProtocol(Protocol):
    """Protocol for binding automatic lease and power management to wrapper class methods"""

    def bind(self, target: Any, actions: Sequence[Callable], passive: bool = False) -> None:
        """Ensure lease availability upon action execution.

        Optionally ensure the system is powered up (if action is not passive).

        This is achieved through decoration. Actions are assumed to be attributes on the target.
        """


class SpotLeashProtocol(Protocol):
    """Protocol for lease and power management."""

    def claim(self) -> bool:
        """Claim the lease.

        Typically implies lease acquisition (aka grabbing) but not necessarily.
        Some implementations may return false if the claim did not fail but
        otherwise had no effect.
        """

    def grab(self, force: bool = False) -> Tuple[bool, Optional[Lease]]:
        """Grab the lease.

        Can be forcefully taken or politely acquired.

        Returns the lease and whether it is a new lease.
        """

    def yield_(self) -> None:
        """Yield the lease.

        Typically implies lease return (aka releasing) but not necessarily.
        Some implementations may use it to collaborate on a lease.
        """

    def release(self) -> None:
        """Release the lease."""

    def tie(self, wrapper: Any) -> SpotLeashContextProtocol:
        """Tie this leash to a wrapper object.

        Useful for method binding.
        """

    @property
    def lease(self) -> Optional[Lease]:
        """Get the latest available lease, if any."""

    @property
    def resources(self) -> List[lease_pb2.LeaseResource]:
        """Get all leased resources in the system."""

    @property
    def async_tasks(self) -> List[AsyncPeriodicQuery]:
        """Get the async tasks this leash needs updated."""


class AsyncLease(AsyncPeriodicQuery):
    """Class to get lease state at regular intervals.  list_leases_async query sent to the robot at every tick.
    Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
        super(AsyncLease, self).__init__("lease", client, logger, period_sec=1.0 / max(rate, 1.0))
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_leases_async()
            callback_future.add_done_callback(lambda future: self._callback(future.result()))
            return callback_future


class SpotLeashContext(SpotLeashContextProtocol):
    """Context for the SpotLeash interface."""

    def __init__(self, target: Any, lease_on_action: bool):
        self._decorator = ClaimAndPowerDecorator(target.power_on, target.claim, lease_on_action)

    def bind(self, target: Any, actions: Sequence[Callable], passive: bool = False) -> None:
        actions = list(actions)
        self._decorator.decorate_functions(target, actions if not passive else [], actions if passive else [])


class SpotLeash(SpotLeashProtocol):
    """Direct, SDK powered leash interface."""

    def __init__(
        self,
        robot: Robot,
        always_take: bool,
        lease_on_action: bool,
        logger: logging.Logger,
        rate: float,
        callback: Callable,
    ) -> None:
        self._robot = robot
        self._always_take = always_take
        self._lease_on_action = lease_on_action
        self._logger = logger

        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        self._lease_task = AsyncLease(self._lease_client, self._logger, rate, callback)
        self._lease_keepalive = None
        self._lease = None

    @property
    def lease(self) -> Optional[Lease]:
        return self._lease

    @property
    def async_tasks(self) -> List[AsyncPeriodicQuery]:
        return [self._lease_task]

    def claim(self) -> bool:
        if self._lease is not None:
            client_name = self._robot.lease_wallet.client_name
            for resource in self._lease_task.proto:
                if resource.resource == "all-leases" and client_name in resource.lease_owner.client_name:
                    return False
        self.grab()
        return True

    def grab(self, force: bool = False) -> Tuple[bool, Optional[Lease]]:
        if self._always_take or force:
            lease = self._lease_client.take()
        else:
            lease = self._lease_client.acquire()
        have_new_lease = self._lease is None or str(lease.lease_proto) != str(self._lease.lease_proto)
        if have_new_lease:
            if self._lease_keepalive is not None:
                self._lease_keepalive.shutdown()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)
            self._lease = lease
        return have_new_lease, self._lease

    def yield_(self) -> None:
        self.release()

    def release(self) -> None:
        if self._lease:
            self._lease_keepalive.shutdown()
            self._lease_keepalive = None
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def tie(self, target: Any) -> SpotLeashContext:
        return SpotLeashContext(target, self._lease_on_action)
