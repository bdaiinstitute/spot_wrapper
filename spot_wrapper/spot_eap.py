import logging
import typing

from bosdyn.api.point_cloud_pb2 import PointCloudRequest
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.common import FutureWrapper
from bosdyn.client.point_cloud import PointCloudClient, build_pc_request


class AsyncPointCloudService(AsyncPeriodicQuery):
    """
    Class to get point cloud at regular intervals.  get_point_cloud_from_sources_async query sent to the robot at
    every tick.  Callback registered to defined callback function.

    """

    def __init__(
        self,
        client: PointCloudClient,
        logger: logging.Logger,
        rate: float,
        callback: typing.Callable,
        point_cloud_requests: PointCloudRequest,
    ) -> None:
        """

        Args:
            client: The Client to a service on the robot
            logger: Logger object
            rate: Rate (Hz) to trigger the query
            callback: Callback function to call when the results of the query are available
            point_cloud_requests
        """
        super(AsyncPointCloudService, self).__init__(
            "robot_point_cloud_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._point_cloud_requests = point_cloud_requests

    def _start_query(self) -> typing.Optional[FutureWrapper]:
        if self._callback and self._point_cloud_requests:
            callback_future = self._client.get_point_cloud_async(
                self._point_cloud_requests
            )
            callback_future.add_done_callback(self._callback)
            return callback_future


class SpotEAP:
    """
    Get pointclouds from the EAP
    """

    def __init__(
        self,
        logger: logging.Logger,
        point_cloud_client: PointCloudClient,
        point_cloud_sources: typing.List[str],
        rate: float = 10,
        callback: typing.Optional[typing.Callable] = None,
    ) -> None:
        """

        Args:
            logger:
            point_cloud_client:
            point_cloud_sources:
            rate:
            callback:
        """
        self._logger = logger
        self._point_cloud_client = point_cloud_client

        self._point_cloud_requests = []
        for source in point_cloud_sources:
            self._point_cloud_requests.append(build_pc_request(source))

        self._point_cloud_task = AsyncPointCloudService(
            self._point_cloud_client,
            self._logger,
            rate,
            callback,
            self._point_cloud_requests,
        )

    @property
    def async_task(self) -> AsyncPeriodicQuery:
        """Returns the async PointCloudService task for the robot"""
        return self._point_cloud_task
