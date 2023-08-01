import logging
import typing
from collections import namedtuple
from dataclasses import dataclass

from bosdyn.api import image_pb2
from bosdyn.client.image import (
    ImageClient,
    build_image_request,
    UnsupportedPixelFormatRequestedError,
)
from bosdyn.client.robot import Robot

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
ImageWithHandBundle = namedtuple(
    "ImageBundle", ["frontleft", "frontright", "left", "right", "back", "hand"]
)

IMAGE_SOURCES_BY_CAMERA = {
    "frontleft": {
        "visual": "frontleft_fisheye_image",
        "depth": "frontleft_depth",
        "depth_registered": "frontleft_depth_in_visual_frame",
    },
    "frontright": {
        "visual": "frontright_fisheye_image",
        "depth": "frontright_depth",
        "depth_registered": "frontright_depth_in_visual_frame",
    },
    "left": {
        "visual": "left_fisheye_image",
        "depth": "left_depth",
        "depth_registered": "left_depth_in_visual_frame",
    },
    "right": {
        "visual": "right_fisheye_image",
        "depth": "right_depth",
        "depth_registered": "right_depth_in_visual_frame",
    },
    "back": {
        "visual": "back_fisheye_image",
        "depth": "back_depth",
        "depth_registered": "back_depth_in_visual_frame",
    },
    "hand": {
        "visual": "hand_color_image",
        "depth": "hand_depth",
        "depth_registered": "hand_depth_in_hand_color_frame",
    },
}

IMAGE_TYPES = {"visual", "depth", "depth_registered"}


@dataclass(frozen=True, eq=True)
class CameraSource:
    """
    Stores information about a camera source configuration

    Attributes:
        camera_name: Name of the camera

        image_types: If non-empty, image requests for this camera will retrieve only the specified image types.
                     Options are visual, depth, or depth_registered.
    """

    camera_name: str
    image_types: typing.List[str]


@dataclass(frozen=True)
class ImageEntry:
    """
    Stores information about a retrieved image

    Attributes:
        camera_name: Name of the camera
        image_type: Type of the image retrieved
        image_response: Protobuf containing the retrieved image
    """

    camera_name: str
    image_type: str
    image_response: image_pb2.ImageResponse


@dataclass()
class ImageQualityConfig:
    """
    Dataclass to store configuration of image quality. Default values are the default for the build_image_request
    """

    DEFAULT_QUALITY = 75.0

    robot_depth_quality: float = DEFAULT_QUALITY
    robot_image_quality: float = DEFAULT_QUALITY
    hand_image_quality: float = DEFAULT_QUALITY
    hand_depth_quality: float = DEFAULT_QUALITY


class SpotImages:
    """
    This module is used to retrieve images from cameras available on the robot. This includes images from the camera
    on the arm, and the built-in cameras on the robot itself.

    To retrieve images, use the various "get" functions.

    TODO: Optionally Retrieve images from the spot cam if it is available
    """

    def __init__(
        self,
        robot: Robot,
        logger: logging.Logger,
        image_client: ImageClient,
        rgb_cameras: bool = True,
        image_quality: ImageQualityConfig = ImageQualityConfig(),
    ) -> None:
        """

        Args:
            robot: Robot object this image module is associated with
            logger: Logger to use
            image_client: Image client to use to retrieve images
            rgb_cameras: If true, the robot model has RGB cameras as opposed to greyscale ones.
        """
        self._robot = robot
        self._logger = logger
        self._rgb_cameras = rgb_cameras
        self._image_client = image_client
        self._image_quality = image_quality

        ############################################
        self._camera_image_requests = []
        for camera_source in CAMERA_IMAGE_SOURCES:
            self._camera_image_requests.append(
                build_image_request(
                    camera_source,
                    image_format=image_pb2.Image.FORMAT_RAW,
                    quality_percent=self._image_quality.robot_image_quality,
                )
            )

        self._depth_image_requests = []
        for camera_source in DEPTH_IMAGE_SOURCES:
            self._depth_image_requests.append(
                build_image_request(
                    camera_source,
                    image_format=image_pb2.Image.FORMAT_RAW,
                    quality_percent=self._image_quality.robot_depth_quality,
                )
            )

        self._depth_registered_image_requests = []
        for camera_source in DEPTH_REGISTERED_IMAGE_SOURCES:
            self._depth_registered_image_requests.append(
                build_image_request(
                    camera_source,
                    image_format=image_pb2.Image.FORMAT_RAW,
                    quality_percent=self._image_quality.robot_depth_quality,
                )
            )

        if self._robot.has_arm():
            self._camera_image_requests.append(
                build_image_request(
                    "hand_color_image",
                    image_format=image_pb2.Image.FORMAT_JPEG,
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8,
                    quality_percent=self._image_quality.hand_image_quality,
                )
            )
            self._depth_image_requests.append(
                build_image_request(
                    "hand_depth",
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
                    quality_percent=self._image_quality.hand_depth_quality,
                )
            )
            self._depth_registered_image_requests.append(
                build_image_request(
                    "hand_depth_in_hand_color_frame",
                    pixel_format=image_pb2.Image.PIXEL_FORMAT_DEPTH_U16,
                    quality_percent=self._image_quality.hand_depth_quality,
                )
            )

        # Build image requests by camera
        self._image_requests_by_camera = {}
        for camera in IMAGE_SOURCES_BY_CAMERA:
            if camera == "hand" and not self._robot.has_arm():
                continue
            self._image_requests_by_camera[camera] = {}
            image_types = IMAGE_SOURCES_BY_CAMERA[camera]
            quality = 75
            for image_type in image_types:
                if image_type.startswith("depth"):
                    image_format = image_pb2.Image.FORMAT_RAW
                    pixel_format = image_pb2.Image.PIXEL_FORMAT_DEPTH_U16
                    if camera == "hand":
                        quality = self._image_quality.hand_depth_quality
                    else:
                        quality = self._image_quality.robot_depth_quality
                else:
                    image_format = image_pb2.Image.FORMAT_JPEG
                    if camera == "hand" or self._rgb_cameras:
                        pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8
                        if camera == "hand":
                            quality = self._image_quality.hand_image_quality
                        else:
                            quality = self._image_quality.robot_image_quality
                    elif camera != "hand":
                        self._logger.info(
                            f"Switching {camera}:{image_type} to greyscale image format."
                        )
                        pixel_format = image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8
                        quality = self._image_quality.robot_image_quality

                source = IMAGE_SOURCES_BY_CAMERA[camera][image_type]
                self._image_requests_by_camera[camera][
                    image_type
                ] = build_image_request(
                    source,
                    image_format=image_format,
                    pixel_format=pixel_format,
                    quality_percent=quality,
                )

    def get_rgb_image(
        self, image_source: str
    ) -> typing.Optional[image_pb2.ImageResponse]:
        """

        Args:
            image_source: Image source from which the image should be retrieved

        Returns:
            ImageResponse protobuf containing the retrieved image, or non if something went wrong.
        Raises:
            ValueError if the image source is invalid
        """
        valid_sources = CAMERA_IMAGE_SOURCES
        valid_sources.append("hand_color_image")
        if image_source not in valid_sources:
            self._logger.warning(
                f"Received request to retrieve rgb image from source {image_source} but it is not a valid source."
            )
            return None
        quality = self._image_quality.robot_image_quality

        if image_source == "hand_color_image":
            if not self._robot.has_arm():
                return None
            quality = self._image_quality.hand_image_quality
        try:
            return self._image_client.get_image(
                [
                    build_image_request(
                        image_source,
                        image_format=image_pb2.Image.FORMAT_RAW,
                        quality_percent=quality,
                    )
                ]
            )[0]
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None

    def get_frontleft_rgb_image(self) -> typing.Optional[image_pb2.ImageResponse]:
        return self.get_rgb_image("frontleft_fisheye_image")

    def get_frontright_rgb_image(self) -> typing.Optional[image_pb2.ImageResponse]:
        return self.get_rgb_image("frontright_fisheye_image")

    def get_left_rgb_image(self) -> typing.Optional[image_pb2.ImageResponse]:
        return self.get_rgb_image("left_fisheye_image")

    def get_right_rgb_image(self) -> typing.Optional[image_pb2.ImageResponse]:
        return self.get_rgb_image("right_fisheye_image")

    def get_back_rgb_image(self) -> typing.Optional[image_pb2.ImageResponse]:
        return self.get_rgb_image("back_fisheye_image")

    def get_hand_rgb_image(self) -> typing.Optional[image_pb2.GetImageResponse]:
        return self.get_rgb_image("hand_color_image")

    def get_images(
        self, image_requests: typing.List[image_pb2.ImageRequest]
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        """
        Get a set of images as specified by the list of requests

        Args:
            image_requests: Request these images from the robot

        Returns:
            ImageBundle containing all the requested images, or none if something went wrong
        """
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError as e:
            self._logger.error(e)
            return None
        if self._robot.has_arm():
            return ImageWithHandBundle(
                frontleft=image_responses[0],
                frontright=image_responses[1],
                left=image_responses[2],
                right=image_responses[3],
                back=image_responses[4],
                hand=image_responses[5],
            )
        else:
            return ImageBundle(
                frontleft=image_responses[0],
                frontright=image_responses[1],
                left=image_responses[2],
                right=image_responses[3],
                back=image_responses[4],
            )

    def get_camera_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        """
        Retrieve all the rgb/greyscale camera images

        Returns:
            ImageBundle containing the images
        """
        return self.get_images(self._camera_image_requests)

    def get_depth_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        """
        Retrieve all the depth images

        Returns:
            ImageBundle containing the images
        """
        return self.get_images(self._depth_image_requests)

    def get_depth_registered_images(
        self,
    ) -> typing.Optional[typing.Union[ImageBundle, ImageWithHandBundle]]:
        """
        Retrieve all the depth registered images

        Returns:
            ImageBundle containing the images
        """
        return self.get_images(self._depth_registered_image_requests)

    def get_images_by_cameras(
        self, camera_sources: typing.List[CameraSource]
    ) -> typing.Optional[typing.List[ImageEntry]]:
        """Calls the GetImage RPC using the image client with requests
        corresponding to the given cameras.
        Args:
           camera_sources: a list of CameraSource objects. There are two
               possibilities for each item in this list. Either it is
               CameraSource(camera='front') or
               CameraSource(camera='front', image_types=['visual', 'depth_registered')
                - If the former is provided, the image requests will include all
                  image types for each specified camera.
                - If the latter is provided, the image requests will be
                  limited to the specified image types for each corresponding
                  camera.
              Note that duplicates of camera names are not allowed.
        Returns:
            a list, where each entry is (camera_name, image_type, image_response)
                e.g. ('frontleft', 'visual', image_response), or none if there was an error
        """
        # Build image requests
        image_requests = []
        source_types = []
        cameras_specified = set()
        for item in camera_sources:
            if item.camera_name in cameras_specified:
                self._logger.error(
                    f"Duplicated camera source for camera {item.camera_name}"
                )
                return None
            image_types = item.image_types
            if image_types is None:
                image_types = IMAGE_TYPES
            for image_type in image_types:
                try:
                    image_requests.append(
                        self._image_requests_by_camera[item.camera_name][image_type]
                    )
                except KeyError:
                    self._logger.error(
                        f"Unexpected camera name '{item.camera_name}' or image type '{image_type}'"
                    )
                    return None
                source_types.append((item.camera_name, image_type))
            cameras_specified.add(item.camera_name)

        # Send image requests
        try:
            image_responses = self._image_client.get_image(image_requests)
        except UnsupportedPixelFormatRequestedError:
            self._logger.error(
                "UnsupportedPixelFormatRequestedError. "
                "Likely pixel_format is set wrong for some image request"
            )
            return None

        # Return
        result = []
        for i, (camera_name, image_type) in enumerate(source_types):
            result.append(
                ImageEntry(
                    camera_name=camera_name,
                    image_type=image_type,
                    image_response=image_responses[i],
                )
            )
        return result
