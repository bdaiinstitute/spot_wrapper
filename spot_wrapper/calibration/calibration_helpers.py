# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

from __future__ import annotations

from typing import Optional, TypedDict

import numpy as np


class Intrinsics(TypedDict):
    dist_coeffs: np.ndarray
    camera_matrix: np.ndarray
    nrows: int
    ncols: int


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
