import typing
import logging

from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.robot import Robot
from bosdyn.client import robot_command
from bosdyn.client.image import (
    ImageClient,
    build_image_request,
    UnsupportedPixelFormatRequestedError,
)
from bosdyn.api import image_pb2

from .spot_config import *


class AsyncImageService(AsyncPeriodicQuery):
    """Class to get images at regular intervals.  get_image_from_sources_async query sent to the robot at every tick.  Callback registered to defined callback function.

    Attributes:
        client: The Client to a service on the robot
        logger: Logger object
        rate: Rate (Hz) to trigger the query
        callback: Callback function to call when the results of the query are available
    """

    def __init__(self, client, logger, rate, callback, image_requests):
        super(AsyncImageService, self).__init__(
            "robot_image_service", client, logger, period_sec=1.0 / max(rate, 1.0)
        )
        self._callback = None
        if rate > 0.0:
            self._callback = callback
        self._image_requests = image_requests

    def _start_query(self):
        if self._callback:
            callback_future = self._client.get_image_async(self._image_requests)
            callback_future.add_done_callback(self._callback)
            return callback_future


class SpotImages:
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
        self._image_client: ImageClient = robot_clients["image_client"]

        self._front_image_requests = []
        for source in front_image_sources:
            self._front_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._side_image_requests = []
        for source in side_image_sources:
            self._side_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._rear_image_requests = []
        for source in rear_image_sources:
            self._rear_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._hand_image_requests = []
        for source in hand_image_sources:
            self._hand_image_requests.append(
                build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW)
            )

        self._front_image_task = AsyncImageService(
            self._image_client,
            self._logger,
            max(0.0, self._rates.get("front_image", 0.0)),
            self._callbacks.get("front_image", None),
            self._front_image_requests,
        )
        self._side_image_task = AsyncImageService(
            self._image_client,
            self._logger,
            max(0.0, self._rates.get("side_image", 0.0)),
            self._callbacks.get("side_image", None),
            self._side_image_requests,
        )
        self._rear_image_task = AsyncImageService(
            self._image_client,
            self._logger,
            max(0.0, self._rates.get("rear_image", 0.0)),
            self._callbacks.get("rear_image", None),
            self._rear_image_requests,
        )
        self._hand_image_task = AsyncImageService(
            self._image_client,
            self._logger,
            max(0.0, self._rates.get("hand_image", 0.0)),
            self._callbacks.get("hand_image", None),
            self._hand_image_requests,
        )

        self.camera_task_name_to_task_mapping: typing.Dict[str, AsyncImageService] = {
            "hand_image": self._hand_image_task,
            "side_image": self._side_image_task,
            "rear_image": self._rear_image_task,
            "front_image": self._front_image_task,
        }

        ############################################
        # TODO: Sort out double publishing of images
        self._camera_image_requests = []
        for camera_source in CAMERA_IMAGE_SOURCES:
            self._camera_image_requests.append(
                build_image_request(
                    camera_source,
                    image_format=image_pb2.Image.FORMAT_JPEG,
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                    quality_percent=50,
                )
            )

        self._depth_image_requests = []
        for camera_source in DEPTH_IMAGE_SOURCES:
            self._depth_image_requests.append(
                build_image_request(
                    camera_source, pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                )
            )

        self._depth_registered_image_requests = []
        for camera_source in DEPTH_REGISTERED_IMAGE_SOURCES:
            self._depth_registered_image_requests.append(
                build_image_request(
                    camera_source, pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                )
            )

    @property
    def front_image_task(self) -> AsyncImageService:
        return self._front_image_task

    @property
    def side_image_task(self) -> AsyncImageService:
        return self._side_image_task

    @property
    def rear_image_task(self) -> AsyncImageService:
        return self._rear_image_task

    @property
    def hand_image_task(self) -> AsyncImageService:
        return self._hand_image_task

    def update_image_tasks(self, image_name: str):
        """Adds an async tasks to retrieve images from the specified image source"""
        task_to_add = self.camera_task_name_to_task_mapping[image_name]

        if task_to_add == self._hand_image_task and not self._robot.has_arm():
            self._logger.warning(
                "Robot has no arm, therefore the arm image task can not be added"
            )
            return

        if task_to_add in self._async_tasks._tasks:
            self._logger.warning(
                f"Task {image_name} already in async task list, will not be added again"
            )
            return

        self._async_tasks.add_task(self.camera_task_name_to_task_mapping[image_name])

    def get_frontleft_rgb_image(self) -> image_pb2.Image:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontleft_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_frontright_rgb_image(self) -> image_pb2.Image:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontright_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_left_rgb_image(self) -> image_pb2.Image:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "left_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_right_rgb_image(self) -> image_pb2.Image:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "right_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_back_rgb_image(self) -> image_pb2.Image:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "back_fisheye_image",
                        pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                        quality_percent=50,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            return None

    def get_images(
        self, image_requests: typing.List[image_pb2.ImageRequest]
    ) -> typing.Optional[ImageBundle]:
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError as e:
            return None
        return ImageBundle(
            frontleft=image_responses[0],
            frontright=image_responses[1],
            left=image_responses[2],
            right=image_responses[3],
            back=image_responses[4],
        )

    def get_camera_images(self) -> ImageBundle:
        return self.get_images(self._camera_image_requests)

    def get_depth_images(self) -> ImageBundle:
        return self.get_images(self._depth_image_requests)

    def get_depth_registered_images(self) -> ImageBundle:
        return self.get_images(self._depth_registered_image_requests)
