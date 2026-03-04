# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

from __future__ import annotations

# import dataclasses
import logging
from dataclasses import dataclass
from typing import Optional, TypedDict

import numpy as np
import numpy.typing as npt
import torch
from jaxtyping import Float, Shaped

logger = logging.getLogger(__name__)

DISTORTION_COEFFICIENT_SIZES = [0, 4, 5, 8, 12, 14]


class CalibrationResults(TypedDict):
    dist_coeffs_origin: np.ndarray
    camera_matrix_origin: np.ndarray
    image_dim_origin: np.ndarray
    dist_coeffs_reference: np.ndarray
    camera_matrix_reference: np.ndarray
    image_dim_reference: np.ndarray
    R: np.ndarray
    T: np.ndarray
    R_handeye: Optional[np.ndarray]
    T_handeye: Optional[np.ndarray]
    average_reprojection_error: float


@dataclass
class Image:
    # Note: this is a jaxtyping-style annotation;
    # when jaxtyping is active (currently only under pytest),
    # the dimensions of the input are bound to the names provided,
    # but only in the context of dynamic type-checking of the
    # generated __init__ method.
    # These same names are later defined as @property methods.
    image_data: Shaped[torch.Tensor, "channels height width"]  # noqa: F722
    """Image tensor with shape (C, H, W), dtype uint8, values in [0,255]"""

    def __post_init__(self) -> None:
        # allow for single channel images
        if len(self.image_data.shape) < 2:
            raise ValueError(f"Not enough dimensions in image data: {self.image_data.shape}. Should be 2d or 3d.")
        elif len(self.image_data.shape) == 2:
            self.image_data = self.image_data.unsqueeze(0)

        elif len(self.image_data.shape) > 3:
            raise ValueError(f"Too many dimensions in image data: {self.image_data.shape}. Should be 2d or 3d.")

    def to_image(self) -> Image:
        return self

    @property
    def device(self):
        return self.image_data.device

    @classmethod
    def from_numpy(
        cls,
        np_image: np.ndarray,
        dtype: torch.dtype | None = None,
        force_copy: bool = True,
    ) -> Image:
        """
        Create an Image from numpy array.

        Note that conventions differ between torch and numpy for how to index an image;
        this accepts a numpy array with channels as the optional last index.

        Because this method uses `torch.Tensor.contiguous()`, it does not interact well
        with multiprocessing, specifically using `fork`. See the README for details.
        """
        if not force_copy:
            logger.warning(f"Cannot pass without copy between numpy and {cls}")

        tensor = torch.as_tensor(np_image, dtype=dtype)

        if len(tensor.shape) == 2:
            tensor = tensor.unsqueeze(-1)

        tensor = tensor.permute((2, 0, 1)).contiguous()

        return cls(tensor)

    @classmethod
    def zeros(cls, width: int, height: int, channels: int) -> Image:
        """Create an all black image."""
        image_data = torch.zeros((channels, height, width))
        return cls.from_tensor(image_data, force_copy=False)

    @property
    def channels(self) -> int:
        """
        Gets the number of channels in the image
        """
        return self.image_data.shape[0]

    @property
    def width(self) -> int:
        """
        Gets the width of the image
        """
        return self.image_data.shape[2]

    @property
    def height(self) -> int:
        """
        Gets the height of the image
        """
        return self.image_data.shape[1]

    @property
    def shape(self) -> tuple[int, int]:
        """
        Gets a tuple of (height, width)
        """
        return self.height, self.width

    @property
    def pixel_count(self) -> int:
        """
        Gets the number of pixels in the image. This is (height * width), _not_ (height * width * channels).
        """
        return self.height * self.width


class CameraIntrinsics:
    """This is an interface to OpenCV's camera model. It contains a 3x3 camera matrix, and a vector of distortion
    coefficients, which must be of length 0, 4, 5, 8, 12 or 14, matching OpenCV's convention. If this vector is too
    short, then it is padded with zeros to the next largest size. If the vector is too long, then an exception is
    thrown.
    (see https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html).

    There are also conversions to and from Open3D's CameraPinholeIntrinsic, with two caveats:
      1. Open3d does _not_ support lens distortion, while CameraIntrinsics does. An exception is thrown if a
         CameraIntrinsics object with a non-zero-length distortion vector is converted to an Open3D representation.
      2. Open3d _does_ support skewed pixels (i.e., camera matrices with non-zero entries in (0,1) and (1,0)). while
         CameraIntrinsics does not. An exception is thrown if a CameraIntrinsics object with any skews.
    """

    # jaxtyping, but not in a dataclass, so they're not checked
    camera_matrix: Float[np.ndarray, "3 3"]  # noqa: F722
    distortion_coeffs: Float[np.ndarray, "*N"]  # noqa: F722 F821

    def __init__(
        self,
        camera_matrix: npt.ArrayLike | None = None,
        distortion_coeffs: npt.ArrayLike | None = None,
        width: int = -1,
        height: int = -1,
    ):
        self.camera_matrix = np.array(camera_matrix) if camera_matrix is not None else np.eye(3)
        self.distortion_coeffs = np.array(distortion_coeffs) if distortion_coeffs is not None else np.empty(0)
        self.width = width
        self.height = height

        if self.camera_matrix.shape != (3, 3):
            raise ValueError(f"Camera matrix must be 3x3, got {self.camera_matrix}.")
        elif not np.allclose(self.camera_matrix[2, :], np.array([0, 0, 1])):
            raise ValueError(f"Camera matrix must be affine, got {self.camera_matrix}.")
        CameraIntrinsics._check_skews(self.camera_matrix)

        if len(self.distortion_coeffs.shape) != 1:
            raise ValueError(f"Distortion coefficients must be Kx1, got {self.distortion_coeffs.shape}")
        elif self.distortion_coeffs.shape[0] > DISTORTION_COEFFICIENT_SIZES[-1]:
            raise ValueError(
                f"Distortion coefficients list too long, got {self.distortion_coeffs.shape} but the maximum length is"
                f" {DISTORTION_COEFFICIENT_SIZES[-1]}."
            )

        if self.distortion_coeffs.shape[0] not in DISTORTION_COEFFICIENT_SIZES:
            for next_greatest_dist_size in DISTORTION_COEFFICIENT_SIZES:
                if next_greatest_dist_size >= self.distortion_coeffs.shape[0]:
                    break

            self.distortion_coeffs = np.pad(
                self.distortion_coeffs, (0, next_greatest_dist_size - self.distortion_coeffs.shape[0]), mode="constant"
            )

    @staticmethod
    def _check_skews(camera_matrix: Float[np.ndarray, "3 3"]):  # noqa: F722
        if not np.allclose(camera_matrix[np.array([0, 1]), np.array([1, 0])], np.zeros(2)):
            raise ValueError(f"Skewed pixels are not supported, got {camera_matrix}.")

    @property
    def fx(self) -> float:
        return self.camera_matrix[0, 0]

    @fx.setter
    def fx(self, val: float) -> None:
        if val <= 0.0:
            raise ValueError(f"Focal lengths must be positive; got {val}.")
        else:
            self.camera_matrix[0, 0] = val

    @property
    def fy(self) -> float:
        return self.camera_matrix[1, 1]

    @fy.setter
    def fy(self, val: float) -> None:
        if val <= 0.0:
            raise ValueError(f"Focal lengths must be positive; got {val}.")
        else:
            self.camera_matrix[1, 1] = val

    @property
    def cx(self) -> float:
        return self.camera_matrix[0, 2]

    @cx.setter
    def cx(self, val: float) -> None:
        if val <= 0.0:
            raise ValueError(f"Principal point values lengths must be positive; got {val}.")
        elif self.width != -1 and val >= self.width:
            raise ValueError(
                f"Principal point values lengths must be less than the corresponding image dimension; got {val} but"
                f" width is {self.width}."
            )
        else:
            self.camera_matrix[0, 2] = val

    @property
    def cy(self) -> float:
        return self.camera_matrix[1, 2]

    @cy.setter
    def cy(self, val: float) -> None:
        if val <= 0.0:
            raise ValueError(f"Principal point values lengths must be positive; got {val}.")
        elif self.height != -1 and val >= self.width:
            raise ValueError(
                f"Principal point values lengths must be less than the corresponding image dimension; got {val} but"
                f" height is {self.height}."
            )
        else:
            self.camera_matrix[1, 2] = val

    def scale(self, scale_factor: float) -> None:
        self.camera_matrix = self.camera_matrix * scale_factor
