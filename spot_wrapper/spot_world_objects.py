import logging
import typing

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot import Robot
from bosdyn.client.world_object import WorldObjectClient


class AsyncWorldObjects(AsyncPeriodicQuery):
    """Class to get world objects.  list_world_objects_async query sent to the robot at every tick.  Callback registered to defined callback function."""

    def __init__(
        self,
        client: WorldObjectClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
    ):
        """
        Args:
            client: Client to the world object service on the robot
            logger: Logger
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
        """
        super(AsyncWorldObjects, self).__init__(
            "world-objects", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self):
        if self._callback:
            callback_future = self._client.list_world_objects_async()
            callback_future.add_done_callback(self._callback)
            return callback_future


class SpotWorldObjects:
    """
    Module which allows access to world objects observed by the robot
    """

    def __init__(
        self,
        logger: logging.Logger,
        world_object_client: WorldObjectClient,
        rate: float = 10,
        callback: typing.Callable = None,
    ):
        """

        Args:
            logger: Logger to use
            world_object_client: Instantiated world object client to use to retrieve world objects
            rate: Rate at which to list objects
            callback: Callback to call with the retrieved objects
        """
        self._logger = logger
        self._world_objects_client = world_object_client
        self._world_objects_task = AsyncWorldObjects(
            self._world_objects_client,
            self._logger,
            rate,
            callback,
        )

    @property
    def async_task(self) -> AsyncWorldObjects:
        """
        The async task used to retrieve world objects periodically
        """
        return self._world_objects_task

    def list_world_objects(self, object_types, time_start_point):
        return self._world_objects_client.list_world_objects(
            object_types, time_start_point
        )
