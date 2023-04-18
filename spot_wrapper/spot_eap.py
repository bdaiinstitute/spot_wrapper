import typing
import logging

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.point_cloud import PointCloudClient, build_pc_request
from bosdyn.client.robot import Robot
from bosdyn.client import robot_command


from .spot_config import *


class AsyncPointCloudService(AsyncPeriodicQuery):
    """
    Class to get point cloud at regular intervals.  get_point_cloud_from_sources_async query sent to the robot at
    every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, point_cloud_requests):
        super(AsyncPointCloudService, self).__init__(
            "robot_point_cloud_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._point_cloud_requests = point_cloud_requests

    def _start_query(self):
        if self._callback and self._point_cloud_requests:
            callback_future = self._client.get_point_cloud_async(
                self._point_cloud_requests
            )
            callback_future.add_done_callback(self._callback)
            return callback_future


class SpotEAP:
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
        self._point_cloud_client: PointCloudClient = robot_clients["point_cloud_client"]

        self._point_cloud_requests = []
        for source in point_cloud_sources:
            self._point_cloud_requests.append(build_pc_request(source))

        self._point_cloud_task = AsyncPointCloudService(
            self._point_cloud_client,
            self._logger,
            max(0.0, self._rates.get("point_cloud", 0.0)),
            self._callbacks.get("lidar_points", None),
            self._point_cloud_requests,
        )

    @property
    def async_task(self) -> AsyncPeriodicQuery:
        """Returns the async PointCloudService task for the robot"""
        return self._point_cloud_task
