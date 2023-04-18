import typing
import logging

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot import Robot
from bosdyn.client.world_object import WorldObjectClient

from .spot_config import *


class AsyncWorldObjects(AsyncPeriodicQuery):
    """Class to get world objects.  list_world_objects_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback):
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
    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        robot_params: typing.Dict[str, typing.Any],
        robot_clients: typing.Dict[str, typing.Any],
    ):
        self._robot = robot
        self._logger = logger
        self._robot_params = robot_params
        self._rates: typing.Dict[str, float] = robot_params["rates"]
        self._callbacks: typing.Dict[str, typing.Callable] = robot_params["callbacks"]
        self._world_objects_client: WorldObjectClient = robot_clients[
            "world_objects_client"
        ]

        self._world_objects_task = AsyncWorldObjects(
            self._world_objects_client,
            self._logger,
            self._rates.get("world_objects", 10.0),
            self._callbacks.get("world_objects", None),
        )

    @property
    def async_task(self):
        return self._world_objects_task

    def list_world_objects(self, object_types, time_start_point):
        return self._world_objects_client.list_world_objects(
            object_types, time_start_point
        )
