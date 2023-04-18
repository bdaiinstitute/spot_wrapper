import typing
import logging

from bosdyn.client.robot import Robot
from bosdyn.client.image import (
    ImageClient,
    build_image_request,
    UnsupportedPixelFormatRequestedError,
)
from bosdyn.api import image_pb2

from .spot_config import *


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
        self._image_client: ImageClient = robot_clients["image_client"]

        ############################################
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
