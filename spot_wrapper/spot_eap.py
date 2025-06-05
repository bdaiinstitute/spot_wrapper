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
        point_cloud_requests: typing.List[PointCloudRequest],
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
        if not isinstance(self._point_cloud_requests, list):
            raise TypeError("Point cloud requests must be a list.")

        if self._callback is not None and len(self._point_cloud_requests) > 0:
            callback_future = self._client.get_point_cloud_async(self._point_cloud_requests)
            callback_future.add_done_callback(lambda future: self._callback(future.result()))
            return callback_future


class SpotEAP:
    """
    Get pointclouds from the EAP
    """

    # Service name for getting pointcloud of VLP16 connected to Spot Core
    POINT_CLOUD_SOURCES = ["velodyne-point-cloud"]

    def __init__(
        self,
        logger: logging.Logger,
        point_cloud_client: PointCloudClient,
        point_cloud_sources: typing.Optional[typing.List[str]] = None,
        rate: float = 10,
        callback: typing.Optional[typing.Callable] = None,
    ) -> None:
        """

        Args:
            logger: Logger to use
            point_cloud_client: Instantiated point cloud client
            point_cloud_sources: Sources from which pointclouds should be retrieved. If not provided, uses default
            rate: Rate at which to retrieve clouds
            callback: Returned clouds will be passed to this callback
        """
        self._logger = logger
        self._point_cloud_client = point_cloud_client

        self._point_cloud_requests = []
        point_cloud_sources = point_cloud_sources or self.POINT_CLOUD_SOURCES
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
