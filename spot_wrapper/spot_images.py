import typing
import logging
from collections import namedtuple

from bosdyn.client.robot import Robot
from bosdyn.client.image import (
    ImageClient,
    build_image_request,
    UnsupportedPixelFormatRequestedError,
)
from bosdyn.api import image_pb2

"""List of body image sources for periodic query"""
CAMERA_IMAGE_SOURCES = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]
DEPTH_IMAGE_SOURCES = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
]
DEPTH_REGISTERED_IMAGE_SOURCES = [
    "frontleft_depth_in_visual_frame",
    "frontright_depth_in_visual_frame",
    "right_depth_in_visual_frame",
    "left_depth_in_visual_frame",
    "back_depth_in_visual_frame",
]
ImageBundle = namedtuple(
    "ImageBundle", ["frontleft", "frontright", "left", "right", "back"]
)


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
                    image_format=image_pb2.Image.FORMAT_RAW,
                )
            )

        self._depth_image_requests = []
        for camera_source in DEPTH_IMAGE_SOURCES:
            self._depth_image_requests.append(
                build_image_request(
                    camera_source, image_format=image_pb2.Image.FORMAT_RAW
                )
            )

        self._depth_registered_image_requests = []
        for camera_source in DEPTH_REGISTERED_IMAGE_SOURCES:
            self._depth_registered_image_requests.append(
                build_image_request(
                    camera_source, image_format=image_pb2.Image.FORMAT_RAW
                )
            )

    def get_frontleft_rgb_image(self) -> image_pb2.ImageResponse:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontleft_fisheye_image",
                        image_format=image_pb2.Image.FORMAT_RAW,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_frontright_rgb_image(self) -> image_pb2.ImageResponse:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "frontright_fisheye_image",
                        image_format=image_pb2.Image.FORMAT_RAW,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_left_rgb_image(self) -> image_pb2.ImageResponse:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "left_fisheye_image", image_format=image_pb2.Image.FORMAT_RAW
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_right_rgb_image(self) -> image_pb2.ImageResponse:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "right_fisheye_image", image_format=image_pb2.Image.FORMAT_RAW
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_back_rgb_image(self) -> image_pb2.ImageResponse:
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        "back_fisheye_image", image_format=image_pb2.Image.FORMAT_RAW
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_images(
        self, image_requests: typing.List[image_pb2.ImageRequest]
    ) -> typing.Optional[ImageBundle]:
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
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
