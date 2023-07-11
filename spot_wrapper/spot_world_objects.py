from __future__ import annotations
import logging
import typing

from bosdyn.api.world_object_pb2 import ListWorldObjectResponse, WorldObjectType
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.common import FutureWrapper
from bosdyn.client.world_object import WorldObjectClient


class AsyncWorldObjects(AsyncPeriodicQuery):
    """
    Class to get world objects. list_world_objects_async query sent to the robot at every tick. Callback
    registered to defined callback function.
    """

    def __init__(
        self,
        client: WorldObjectClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Optional[typing.Callable] = None,
    ) -> None:
        """
        Args:
            client: Client to the world object service on the robot
            logger: Logger
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
        """
        super().__init__(
            "world-objects", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback

    def _start_query(self) -> typing.Optional[FutureWrapper]:
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
        callback: typing.Optional[typing.Callable] = None,
    ) -> None:
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

    def list_world_objects(
        self, object_types: typing.List[WorldObjectType], time_start_point: float
    ) -> ListWorldObjectResponse:
        """
        Get a list of world objects with the specified types which were seen after the given time point

        Args:
            object_types: List of object types which should be retrieved
            time_start_point: Objects observed after this time will be filtered out of the response

        Returns:
            List world object response containing the filtered list of world objects

        """
        return self._world_objects_client.list_world_objects(
            object_types, time_start_point
        )
