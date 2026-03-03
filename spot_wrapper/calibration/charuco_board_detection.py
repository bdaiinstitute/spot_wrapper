# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import logging
import time
from copy import deepcopy
from math import radians
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from calibration_helpers import CalibrationResults

logging.basicConfig(
    level=logging.INFO,
)

logger = logging.getLogger(__name__)

SPOT_DEFAULT_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
OPENCV_VERSION = tuple(map(int, cv2.__version__.split(".")))
OPENCV_CHARUCO_LIBRARY_CHANGE_VERSION = (4, 7, 0)


def create_charuco_board(
    num_checkers_width: int,
    num_checkers_height: int,
    checker_dim: float,
    marker_dim: float,
    aruco_dict: cv2.aruco_Dictionary,
    legacy: bool = True,
) -> cv2.aruco_CharucoBoard:
    """
    Create a Charuco board using the provided parameters and Aruco dictionary.
    Issues a deprecation warning if using the older 'CharucoBoard_create' method.

    Args:
        num_checkers_width (int): Number of checkers along the width of the board.
        num_checkers_height (int): Number of checkers along the height of the board.
        checker_dim (float): Size of the checker squares.
        marker_dim (float): Size of the Aruco marker squares.
        aruco_dict (cv2.aruco_Dictionary): The Aruco dictionary to use for marker generation.

    Returns:
        charuco (cv2.aruco_CharucoBoard): The generated Charuco board.
    """

    opencv_version = tuple(map(int, cv2.__version__.split(".")))

    if opencv_version < (4, 7, 0):
        logger.warning(
            (
                "Creating Charuco Board..."
                "You're using an older version of OpenCV requires the additional OpenCV Modules. "
                "This will not work without the additional modules (opencv-contrib-python). "
                "Consider upgrading to OpenCV >= 4.7.0 to enable "
                "the use of this tool with base opencv (opencv-python) "
                "without relying on additional modules."
            ),
        )

        # Create Charuco board using the older method
        charuco = cv2.aruco.CharucoBoard_create(
            num_checkers_width,
            num_checkers_height,
            checker_dim,
            marker_dim,
            aruco_dict,
        )

    else:
        # Create Charuco board using the newer method
        charuco = cv2.aruco_CharucoBoard(
            (num_checkers_width, num_checkers_height),
            checker_dim,
            marker_dim,
            aruco_dict,
        )
        charuco.setLegacyPattern(legacy)

    return charuco


SPOT_DEFAULT_CHARUCO = create_charuco_board(
    num_checkers_width=9,
    num_checkers_height=4,
    checker_dim=0.115,
    marker_dim=0.09,
    aruco_dict=SPOT_DEFAULT_ARUCO_DICT,
    legacy=True,
)


def multistereo_calibration_charuco(
    images: np.ndarray,
    desired_stereo_pairs: Optional[List[Tuple[int, int]]] = None,
    charuco_board: cv2.aruco_CharucoBoard = SPOT_DEFAULT_CHARUCO,
    aruco_dict: cv2.aruco_Dictionary = SPOT_DEFAULT_ARUCO_DICT,
    camera_matrices: Optional[Dict] = {},
    dist_coeffs: Optional[Dict] = {},
    poses: Union[np.ndarray, None] = None,
) -> Dict:
    """
    Calibrates the intrinsic and extrinsic parameters for multiple stereo camera pairs
    using Charuco board markers. This function performs stereo calibration on a series of
    time-synchronized images
    captured by multiple cameras. It uses a Charuco board for calibration and returns
    the calibrated intrinsic and extrinsic parameters for the specified stereo camera pairs.

    Args:
        images (np.ndarray): A 2D array of time-synchronized camera captures. The array should
            be of shape (n, m), where 'n' represents the number of time-synchronized captures
            and 'm' represents the number of cameras. Each internal list corresponds to a set
            of images captured at the same time across multiple cameras.

        desired_stereo_pairs (Optional[List[Tuple[int, int]]], optional): A list of tuples
            specifying the pairs of camera indices to be calibrated. The indices refer to the
            cameras based on their positions in the `images` array. For example, index `0`
            corresponds to the first camera, index `1` to the second camera, and so forth,
            up to `n-1` (where `n` is the number of cameras).
            If None, all cameras are registered to the first camera (index `0`). Defaults to None.

            The relationship between `desired_stereo_pairs` and `images` is crucial. Each tuple in
            `desired_stereo_pairs` should be in the form
            `(origin_camera_idx, reference_camera_idx)`,
            where `origin_camera_idx` and `reference_camera_idx` correspond to the
            indices of the cameras in the `images` array. For each stereo pair,
            the function attempts to calibrate the `origin_camera_idx`
            relative to the `reference_camera_idx`. If no pairs are provided,
            the function defaults to registering
            all cameras to the first camera (index `0`).

        charuco_board (cv2.aruco_CharucoBoard, optional): The Charuco board configuration used
            for calibration. Defaults to SPOT_DEFAULT_CHARUCO.

        aruco_dict (cv2.aruco_Dictionary, optional): The Aruco dictionary used
            to detect the markers. Defaults to SPOT_DEFAULT_ARUCO_DICT.

        camera_matrices (Optional[Dict], optional): A dictionary mapping camera indices
            to their existing camera matrices. If a matrix is not provided for a camera,
            it will be computed during calibration
            if that camera is part of a stereo pair. Defaults to an empty dictionary.

        dist_coeffs (Optional[Dict], optional): A dictionary mapping camera indices
            to their distortion coefficients.
            If coefficients are not provided for a camera, they will be computed during calibration.
            Defaults to an empty dictionary.
        poses (Union[np.ndarray, None]): Either a list of 4x4 homogenous transforms from which
            pictures where taken, or None if unknown. Needs to be supplied for robot to camera cal.
            (planning frame to base frame), or None

    Raises:
        ValueError: Raised if fewer than two cameras are provided, as stereo calibration
            requires at least two cameras.
        ValueError: Raised if a stereo pair includes the same camera twice.
        ValueError: Raised if a camera in a stereo pair cannot be calibrated.

    Returns:
        Dict: A dictionary containing the calibrated intrinsic and extrinsic parameters
            for the specified stereo pairs.
        The keys are formatted as "origin_camera_idx_reference_camera_idx",
            and the values contain the calibration data.

    """
    calibration = {}
    num_cams = images.shape[1]
    if num_cams == 0 or num_cams == 1:
        raise ValueError("Stereo Madness requires more than one camera. Use the monocular method instead.")
    else:  # More than one camera - Multi-Stereo Madness ;p
        for idx in range(num_cams):
            if idx not in camera_matrices:
                camera_matrices[idx] = None
            if idx not in dist_coeffs:
                dist_coeffs[idx] = None

        if desired_stereo_pairs is None:
            desired_stereo_pairs = []
            for idx in range(1, num_cams):
                """
                Desired stereo pairs is a mapping, where the 0th index is the origin (base)
                frame, where the reference frame is defined within the origin (base) frame's
                coordinate system. The reference frame is the 1st index.

                By Default, if desired_stereo_pairs is None,
                then each camera's intrinsic is defined in the 0th camera's frame,
                so this is set as the origin for all stereo calibrations
                """
                desired_stereo_pairs.append((0, idx))  # just register all in 0th cam frame

        for idx, pair in enumerate(desired_stereo_pairs):
            for index in pair:
                if index < 0 or index > num_cams:
                    raise ValueError(
                        f"Requested to stereo register non-existent camera index {index} out of {num_cams}"
                    )
            origin_camera_idx = pair[0]
            reference_camera_idx = pair[1]

            if pair[0] == pair[1]:
                logging.warning(
                    "You requested to register a camera to itself. Ignoring,and continuing to the next stereo pair."
                )
                continue
            logging.info(
                f"Attempting to register {origin_camera_idx} "
                f"to {reference_camera_idx}, pair {idx} of "
                f" {len(desired_stereo_pairs)}"
            )
            try:
                key = str(origin_camera_idx) + "_" + str(reference_camera_idx)
                stereo_dict = stereo_calibration_charuco(
                    parent_images={str(i): img for i, img in enumerate(images[:, origin_camera_idx])},
                    child_images={str(i): img for i, img in enumerate(images[:, reference_camera_idx])},
                    charuco_board=charuco_board,
                    aruco_dict=aruco_dict,
                    camera_matrix_parent=camera_matrices[origin_camera_idx],
                    dist_coeffs_parent=dist_coeffs[origin_camera_idx],
                    camera_matrix_child=camera_matrices[reference_camera_idx],
                    dist_coeffs_child=dist_coeffs[reference_camera_idx],
                    poses=poses,
                )
                camera_matrices[origin_camera_idx] = stereo_dict["camera_matrix_origin"]
                dist_coeffs[origin_camera_idx] = stereo_dict["dist_coeffs_origin"]
                camera_matrices[reference_camera_idx] = stereo_dict["camera_matrix_reference"]
                dist_coeffs[reference_camera_idx] = stereo_dict["dist_coeffs_reference"]

                calibration[key] = stereo_dict
            except ValueError as ve:  # maybe could keep going here instead for partial calibration?
                raise ValueError(f"Failed to calibrate pair {pair} : {ve}")

    return calibration


def get_correlation_map_to_enforce_ascending_ids_from_bottom_left_corner(
    all_charuco_corners: List,
    all_charuco_ids: List,
    charuco_board: cv2.aruco_CharucoBoard,
) -> List:
    """
    This is needed only for OpenCV versions > 4.7.0. Basically,
    this determines the needed correlation
    to ensure that internal corners are numbered left to right bottom
    up as opposed to left to right top down.

    For more info see https://github.com/opencv/opencv/issues/26126

    Args:
        all_charuco_corners (List): All charuco corners of the board.
        all_charuco_ids (List): All charuco ids of the board, as returned by the Charuco detector
            for an ideal board with a full view. Expected to be a list ascending
            from 0 but could be otherwise
        charuco_board (cv2.aruco_CharucoBoard): the charuco board to utilize
            to construct the correlation map

    Returns:
        List: the correlation map where the indicies represent to the original
            ordering of corners, and the values at each index represent the new
            ordering index of a corner. Can be used to ensure bottom up
            left to right ordering of internal corners.
    """

    num_checker_width, num_checker_height = charuco_board.getChessboardSize()
    num_interior_corners = (num_checker_width - 1) * (num_checker_height - 1)
    correlation_map = np.array([i for i in range(num_interior_corners)])
    # Adjust indexing to account for nested arrays
    first_corner_height = all_charuco_corners[all_charuco_ids[0][0]][0][1]
    last_corner_height = all_charuco_corners[all_charuco_ids[-1][0]][0][1]
    row_num_a = 0
    row_num_b = num_checker_height - 2

    if first_corner_height < last_corner_height:
        logger.warning(
            "Detected Charuco Configuration "
            "where internal corners (detected checker corners) are numbered top down, "
            "(left to right) as opposed to bottom up (left to right). "
            "Enforcing bottom up numbering instead. "
            "This ensures that the Z axis points out of the board "
            "As opposed to -180 degrees about the X axis "
            "relative to the Z out of the board"
        )
    else:
        logger.warning(
            "You have selected to enforce ascending ids from the bottom left corner "
            "But it seems as if your ids are already in that form "
            "Returning identity correlation map"
        )
        return [int(mapping) for mapping in correlation_map]

    while row_num_a < row_num_b:
        row_num_a_correlation_slice = slice(
            row_num_a * (num_checker_width - 1), (row_num_a * (num_checker_width - 1) + (num_checker_width - 1)), 1
        )

        row_num_b_correlation_slice = slice(
            row_num_b * (num_checker_width - 1), ((row_num_b * (num_checker_width - 1)) + (num_checker_width - 1)), 1
        )
        # Record A
        precopy_row_a = deepcopy(correlation_map[row_num_a_correlation_slice])
        # Copy B onto A
        correlation_map[row_num_a_correlation_slice] = correlation_map[row_num_b_correlation_slice]
        # copy old A into B
        correlation_map[row_num_b_correlation_slice] = precopy_row_a
        row_num_a += 1
        row_num_b -= 1
    return [int(mapping) for mapping in correlation_map]


def detect_charuco_corners(
    img: np.ndarray,
    charuco_board: cv2.aruco_CharucoBoard,
    aruco_dict: cv2.aruco_Dictionary,
    enforce_ascending_ids_from_bottom_left_corner: Union[bool, None] = None,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Detect the Charuco Corners and their IDs in an image, with support for OpenCV versions before and after 4.7.0.

    Args:
        img (np.ndarray): The image to find Charuco corners in.
        charuco_board (cv2.aruco_CharucoBoard, optional): The Charuco board to look for.
            Defaults to SPOT_DEFAULT_CHARUCO.
        aruco_dict (cv2.aruco_Dictionary, optional): The Aruco dictionary to use.
            Defaults to SPOT_DEFAULT_ARUCO_DICT.
        enforce_ascending_ids_from_bottom_left_corner (Union[bool, None]): whether to
            ensure that internal charuco corners are numbered left to right bottom up
            (only ensures bottom up, assumes already left to right).

    Returns:
        Tuple[Optional[np.ndarray], Optional[np.ndarray]]: The detected corners and their IDs,
            or (None, None) if not found.
    """
    # Convert the image to grayscale if it's not already
    if len(img.shape) == 2:
        gray = img
    else:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if OPENCV_VERSION < OPENCV_CHARUCO_LIBRARY_CHANGE_VERSION:
        # Older OpenCV version
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
        if ids is not None:
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, charuco_board, minMarkers=0
            )
            return charuco_corners, charuco_ids
        else:
            return None, None
    else:
        # Newer OpenCV version with charuco_detector
        detector_params = cv2.aruco.CharucoParameters()
        detector_params.minMarkers = 0
        detector_params.tryRefineMarkers = True
        charuco_detector = cv2.aruco.CharucoDetector(charuco_board, detector_params)
        charuco_detector.setBoard(charuco_board)
        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)

        if charuco_ids is None:
            return None, None

        enforce_ids = enforce_ascending_ids_from_bottom_left_corner
        if enforce_ids is not None and hasattr(detect_charuco_corners, "enforce_ids"):
            logger.warning(
                "Previously, for detecting internal charuco corners, the enforce "
                "ascending from bottom left corner id policy was set to: "
                f"{detect_charuco_corners.enforce_ids}"
                f"it will now be set to {enforce_ids}"
            )
            detect_charuco_corners.enforce_ids = enforce_ids
        elif enforce_ids is not None and not hasattr(detect_charuco_corners, "enforce_ids"):
            logger.warning(
                "For detecting charuco corners, the enforce "
                "ascending from bottom left corner id policy is set to: "
                f"{enforce_ids}. For this call, and future calls until set otherwise."
            )
            detect_charuco_corners.enforce_ids = enforce_ids

        # Create the identity correlation map...
        num_checker_width, num_checker_height = charuco_board.getChessboardSize()
        num_interior_corners = (num_checker_width - 1) * (num_checker_height - 1)
        correlation_map = np.array([i for i in range(num_interior_corners)])

        if (
            hasattr(detect_charuco_corners, "enforce_ids")
            and detect_charuco_corners.enforce_ids
            and not hasattr(detect_charuco_corners, "corr_map")
        ):  # correlation map not computed
            ideal_charuco = charuco_board.generateImage(outSize=(1000, 1000))
            all_charuco_corners, all_charuco_ids, _, _ = charuco_detector.detectBoard(ideal_charuco)

            detect_charuco_corners.corr_map = get_correlation_map_to_enforce_ascending_ids_from_bottom_left_corner(
                all_charuco_corners, all_charuco_ids, charuco_board
            )

        if (
            hasattr(detect_charuco_corners, "enforce_ids")
            and detect_charuco_corners.enforce_ids
            and hasattr(detect_charuco_corners, "corr_map")
        ):  # correlation map computed
            logger.warning("Using cached correlation map to order IDs")
            correlation_map = detect_charuco_corners.corr_map  # grab the map

        reworked_charuco_ids = []
        for charuco_id in charuco_ids:
            reworked_charuco_ids.append([correlation_map[charuco_id[0]]])

        return charuco_corners, reworked_charuco_ids


def get_charuco_board_object_points(
    charuco_board: cv2.aruco_CharucoBoard,
    corners_ids: Union[List, np.ndarray],
) -> np.ndarray:
    """
    From a charuco board, and corner ids, generate the object points for use
    in a calibration sequence to solve the perspective and point problem.

    Args:
        charuco_board (cv2.aruco_CharucoBoard): the charuco board
            to generate object points for
        corners_ids (Union[List, np.ndarray]): which corner ids to generate points for

    Returns:
        np.ndarray: the object points
    """
    if OPENCV_VERSION < OPENCV_CHARUCO_LIBRARY_CHANGE_VERSION:
        corners = charuco_board.chessboardCorners
    else:
        corners = charuco_board.getChessboardCorners()
    object_points = []
    for idx in corners_ids:
        object_points.append(corners[idx])
    return np.array(object_points, dtype=np.float32)


def calibrate_single_camera_charuco(
    images: List[np.ndarray],
    charuco_board: cv2.aruco_CharucoBoard = SPOT_DEFAULT_CHARUCO,
    aruco_dict: cv2.aruco_Dictionary = SPOT_DEFAULT_ARUCO_DICT,
    debug_str: str = "",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Calibrate a monocular camera from a charuco board.

    Args:
        images (List[np.ndarray]): A list of images of the board
        charuco_board (cv2.aruco_CharucoBoard, optional): which
            board to look for in the images. Defaults to SPOT_DEFAULT_CHARUCO.
        aruco_dict (cv2.aruco_Dictionary, optional): which
            aruco dict to look for in the images. Defaults to SPOT_DEFAULT_ARUCO_DICT.
        debug_str (str, optional): what to add to the logging
            to differentiate this monocular calibration from others, for the sake
            of stereo calibrations. Defaults to "".

    Raises:
        ValueError: Not enough data to calibrate

    Returns:
        Tuple[np.ndarray, np.ndarray]: the camera matrix, distortion coefficients,
        rotation matrices, tvecs
    """
    all_corners = []
    all_ids = []
    img_size = None
    for idx, img in enumerate(images):
        if img_size is None:
            img_size = img.shape[:2][::-1]

        charuco_corners, charuco_ids = detect_charuco_corners(img, charuco_board, aruco_dict)

        if charuco_corners is not None and len(charuco_corners) >= 8:
            all_corners.append(charuco_corners)
            all_ids.append(charuco_ids)
        else:
            logger.warning(f"Not enough corners detected in image index {idx} {debug_str}; ignoring")

    if len(all_corners) > 1:
        obj_points_all = []
        img_points = []
        ids = []
        for corners, ids in zip(all_corners, all_ids):
            obj_points = get_charuco_board_object_points(charuco_board, ids)
            obj_points_all.append(obj_points)
            img_points.append(corners)
        logger.info(f"About to process {len(obj_points_all)} points for single camera cal | {debug_str}")
        # Long term improvement: could pull tvec for use to localize camera relative to robot?
        _, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(  # use LU flag critical for speed
            obj_points_all,
            img_points,
            img_size,
            None,
            None,
            flags=cv2.CALIB_USE_LU,
        )
        rmats = np.array([cv2.Rodrigues(rvec)[0] for rvec in rvecs])
        return camera_matrix, dist_coeffs, rmats, tvecs
    else:
        raise ValueError(f"Not enough valid points to individually calibrate {debug_str}")


def stereo_calibration_charuco(
    parent_images: Dict[str, np.ndarray],
    child_images: Dict[str, np.ndarray],
    charuco_board: cv2.aruco_CharucoBoard = SPOT_DEFAULT_CHARUCO,
    aruco_dict: cv2.aruco_Dictionary = SPOT_DEFAULT_ARUCO_DICT,
    camera_matrix_parent: Optional[np.ndarray] = None,
    dist_coeffs_parent: Optional[np.ndarray] = None,
    camera_matrix_child: Optional[np.ndarray] = None,
    dist_coeffs_child: Optional[np.ndarray] = None,
    poses: Union[np.ndarray, None] = None,
) -> CalibrationResults:
    """
    Perform a stereo calibration from a set of synchronized stereo images of a charuco calibration
    board.

    Args:
        parent_images (List[np.ndarray]): A list of images synced to reference images from camera 0
        child_images (List[np.ndarray]): A list of images synced to origin images from camera 1
        charuco_board (cv2.aruco_CharucoBoard, optional): What charuco board to
            use for the cal. Defaults to SPOT_DEFAULT_CHARUCO.
        aruco_dict (cv2.aruco_Dictionary, optional): What aruco board to use for the cal.
            Defaults to SPOT_DEFAULT_ARUCO_DICT.
        camera_matrix_parent (Optional[np.ndarray], optional): What camera
            matrix to assign to camera 0. If none, is computed. Defaults to None.
        dist_coeffs_parent (Optional[np.ndarray], optional): What distortion coefficients
            to assign to camera 0. If None, is computed. Defaults to None.
        projection_matrix_origin (Optional[np.ndarray], optional): What projection
            matrix to assign to camera 0. If None, is computed. Defaults to None.
        camera_matrix_child (Optional[np.ndarray], optional): What camera
            matrix to assign to camera 1. If none, is computed. . Defaults to None.
        dist_coeffs_child (Optional[np.ndarray], optional): What distortion coefficients
            to assign to camera 1. If None, is computed. Defaults to None.
        poses (Union[np.ndarray, None]): Either a list of 4x4 homogenous transforms from which
            pictures where taken, or None if unknown. Needs to be supplied for robot to camera cal.
            (planning frame to base frame), or None
    Raises:
        ValueError: Could not calibrate a camera individually
        ValueError: Not enough points to stereo calibrate
        ValueError: Could not stereo calibrate

    Returns:
        Dict: _description_
    """
    camera_child: np.ndarray = np.eye(3)
    coeffs_child: np.ndarray = np.zeros((5,))
    camera_parent: np.ndarray = np.eye(3)
    coeffs_parent: np.ndarray = np.zeros((5,))

    # Pre-filter parent images to frames with enough corners, and simultaneously
    # filter poses so they remain aligned with the rmats/tvecs returned by
    # calibrate_single_camera_charuco (which silently drops frames with < 8 corners).
    valid_parent_imgs: Dict[str, np.ndarray] = {}
    poses_aligned_with_parent: Optional[List] = [] if poses is not None else None
    for i, (fname, img) in enumerate(parent_images.items()):
        corners, _ = detect_charuco_corners(img, charuco_board, aruco_dict)
        if corners is not None and len(corners) >= 8:
            valid_parent_imgs[fname] = img
            if poses is not None:
                poses_aligned_with_parent.append(poses[i])

    # first do single camera cal for parent. rotation and tvecs are used, and camera matrix, dist coeffs
    logger.info("Calibrating Parent Camera individually")
    (camera_matrix_parent_new, dist_coeffs_parent_new, rmats_origin, tvecs_origin) = calibrate_single_camera_charuco(
        images=valid_parent_imgs.values(),
        charuco_board=charuco_board,
        aruco_dict=aruco_dict,
        debug_str="for parent camera",
    )

    if camera_matrix_parent_new is None:
        raise ValueError("Could not obtain cam matrix parent camera to charuco board, needed for pose")
    if dist_coeffs_parent_new is None:
        raise ValueError("Could not obtain dist coeffs of parent camera to charuco board, needed for pose")

    # if None passed in, use these newly computed values
    if camera_matrix_parent is None:
        camera_parent = camera_matrix_parent_new
    else:
        camera_parent = camera_matrix_parent
    if dist_coeffs_parent is None:
        coeffs_parent = dist_coeffs_parent_new
    else:
        coeffs_parent = dist_coeffs_parent

    # next do single camera cal for child. rotation and tvecs are used, and camera matrix, dist coeffs
    start_time = time.perf_counter()
    logger.info("Calibrating Child Camera individually")
    (camera_matrix_child_new, dist_coeffs_child_new, _, _) = calibrate_single_camera_charuco(
        images=child_images.values(),
        charuco_board=charuco_board,
        aruco_dict=aruco_dict,
        debug_str="for child camera",
    )
    elapsed_time = time.perf_counter() - start_time
    logger.info(f"Finished calibration child camera in {elapsed_time:.4f} seconds")

    if camera_matrix_child_new is None:
        raise ValueError("Could not obtain cam matrix child camera to charuco board, needed for pose")
    if dist_coeffs_child_new is None:
        raise ValueError("Could not obtain dist coeffs of child camera to charuco board, needed for pose")

    logger.info("Finished individual camera calibrations, starting stereo calibration")

    # if None passed in, use these newly computed values
    if camera_matrix_child is None:
        camera_child = camera_matrix_child_new
    else:
        camera_child = camera_matrix_child
    if dist_coeffs_child is None:
        coeffs_child = dist_coeffs_child_new
    else:
        coeffs_child = dist_coeffs_child

    all_corners_parent = []
    all_corners_child = []
    all_ids_parent = []
    all_ids_child = []
    parent_img_size = None
    child_img_size = None

    # let gets corresponding images
    parent_keys = set(parent_images.keys())
    child_keys = set(child_images.keys())
    common_keys = parent_keys.intersection(child_keys)

    logger.info("Collecting shared points for stereo calibration")
    if len(common_keys) == 0:
        raise ValueError("No common images between parent and child cameras for stereo calibration")

    for fname in list(common_keys):
        origin_img = parent_images[fname]
        reference_img = child_images[fname]
        if parent_img_size is None:
            parent_img_size = origin_img.shape[:2][::-1]
        if child_img_size is None:
            child_img_size = reference_img.shape[:2][::-1]

        origin_charuco_corners, origin_charuco_ids = detect_charuco_corners(origin_img, charuco_board, aruco_dict)
        reference_charuco_corners, reference_charuco_ids = detect_charuco_corners(
            reference_img, charuco_board, aruco_dict
        )

        if origin_charuco_corners is not None and reference_charuco_corners is not None:
            all_corners_parent.append(origin_charuco_corners)
            all_corners_child.append(reference_charuco_corners)
            all_ids_parent.append(origin_charuco_ids)
            all_ids_child.append(reference_charuco_ids)

    if len(all_corners_parent) == 0:
        raise ValueError("Not enough shared points for stereo calibration.")
    logger.info("about to find common ids...")
    obj_points_all = []
    img_points_origin = []
    img_points_reference = []
    for (
        origin_corners,
        reference_corners,
        origin_ids,
        reference_ids,
    ) in zip(
        all_corners_parent,
        all_corners_child,
        all_ids_parent,
        all_ids_child,
        strict=True,
    ):
        common_ids = np.intersect1d(origin_ids, reference_ids)  # sorted flat array
        if len(common_ids) >= 6:  # Ensure there are at least 6 points
            obj_points = get_charuco_board_object_points(charuco_board, common_ids)
            # Build id->corner-index maps so we can look up corners in the same
            # sorted order as common_ids / obj_points. Using np.isin preserves
            # the original detection order, which may differ from intersect1d's
            # sorted order, causing a 2D-3D correspondence mismatch → NaN R/T.
            origin_id_to_idx = {int(np.ravel(id_)[0]): i for i, id_ in enumerate(origin_ids)}
            reference_id_to_idx = {int(np.ravel(id_)[0]): i for i, id_ in enumerate(reference_ids)}
            selected_origin = np.array(
                [origin_corners[origin_id_to_idx[int(cid)]] for cid in common_ids], dtype=np.float32
            )
            selected_reference = np.array(
                [reference_corners[reference_id_to_idx[int(cid)]] for cid in common_ids], dtype=np.float32
            )
            obj_points_all.append(obj_points)
            img_points_origin.append(selected_origin)
            img_points_reference.append(selected_reference)

    # sanity check
    if len(obj_points_all) == 0:
        raise ValueError("Not enough valid points for stereo calibration.")

    logger.info(
        f"Collected {len(obj_points_all)} shared point sets for stereo calibration. Starting stereo calibration..."
    )
    # Compute the stereo extrinsic (parent-to-child) via per-view PnP rather than
    # cv2.stereoCalibrate, which is prone to NaN when the two cameras have very
    # different resolutions or when the IR/depth images produce noisy detections
    # that destabilise the Essential-matrix initialisation used internally by
    # stereoCalibrate.  Per-view PnP + SVD averaging is more robust.
    start_time = time.perf_counter()
    R_rel_list: List[np.ndarray] = []
    T_rel_list: List[np.ndarray] = []
    reproj_errors: List[float] = []
    for op, ip_parent, ip_child in zip(obj_points_all, img_points_origin, img_points_reference):
        ok_p, rvec_p, tvec_p = cv2.solvePnP(op, ip_parent, camera_parent, coeffs_parent, flags=cv2.SOLVEPNP_ITERATIVE)
        ok_c, rvec_c, tvec_c = cv2.solvePnP(op, ip_child, camera_child, coeffs_child, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok_p or not ok_c:
            logger.warning("solvePnP failed for a view; skipping.")
            continue
        R_p, _ = cv2.Rodrigues(rvec_p)
        R_c, _ = cv2.Rodrigues(rvec_c)
        # child_R_parent, child_T_parent  (X_child = R @ X_parent + T)
        R_i = R_c @ R_p.T
        T_i = tvec_c.flatten() - R_i @ tvec_p.flatten()
        R_rel_list.append(R_i)
        T_rel_list.append(T_i)
        # per-view reprojection error in parent camera for a rough quality metric
        proj, _ = cv2.projectPoints(op, rvec_p, tvec_p, camera_parent, coeffs_parent)
        reproj_errors.append(float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - ip_parent.reshape(-1, 2), axis=1))))

    if len(R_rel_list) < 3:
        raise ValueError(
            f"Only {len(R_rel_list)} views produced valid PnP solutions; need at least 3 for stereo extrinsic."
        )

    T_arr = np.array(T_rel_list)  # (N, 3)
    R_arr = np.array(R_rel_list)  # (N, 3, 3)

    # --- outlier rejection on T ---
    # Each T_i = t_child - R_i @ t_parent.  Individual tvec values can be O(1 m),
    # so a few noisy IR-camera PnP solutions cause catastrophic cancellation and
    # produce wildly large T_i.  Use the per-component MAD to find and discard them.
    T_median = np.median(T_arr, axis=0)
    abs_dev = np.abs(T_arr - T_median)  # (N, 3)
    mad = np.median(abs_dev, axis=0)  # (3,)
    mad = np.where(mad < 1e-9, 1e-9, mad)  # avoid division by zero
    # A view is an inlier only if every component is within 5 MADs of the median
    inlier_mask = np.all(abs_dev / mad < 5.0, axis=1)
    n_inliers = int(inlier_mask.sum())
    if n_inliers < 3:
        logger.warning(f"Outlier rejection left only {n_inliers} T inliers; using all {len(T_arr)} views.")
        inlier_mask = np.ones(len(T_arr), dtype=bool)

    T_inliers = T_arr[inlier_mask]
    R_inliers = R_arr[inlier_mask]
    reproj_inliers = [e for e, ok in zip(reproj_errors, inlier_mask) if ok]
    logger.info(f"Stereo extrinsic: {n_inliers}/{len(T_arr)} views kept after outlier rejection.")

    # Robust translation: median of inliers
    T = np.median(T_inliers, axis=0).reshape(3, 1)

    # Average rotation matrices over inliers and project back to SO(3) via SVD
    R_mean = np.mean(R_inliers, axis=0)
    U, _, Vt = np.linalg.svd(R_mean)
    R = U @ Vt
    if np.linalg.det(R) < 0:  # fix improper rotation (reflection)
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt

    err = float(np.mean(reproj_inliers))  # mean per-view reprojection error (parent camera)
    elapsed_time = time.perf_counter() - start_time
    logger.info(
        f"Stereo extrinsic estimated from {n_inliers} inlier views in {elapsed_time:.4f} s, "
        f"mean parent reproj error: {err:.3f} px."
    )

    # now we will estimate these
    camera_to_robot_R = np.eye(3)  # Extract rotation
    camera_to_robot_T = np.eye(3)  # Extract translation

    if poses is not None:
        logger.info("Attempting hand-eye calibation....")
        # poses_aligned_with_parent was pre-filtered above to match rmats_origin/tvecs_origin
        filtered_poses = np.array(poses_aligned_with_parent)
        # Use the filtered poses for the target-to-camera transformation
        R_gripper2base = np.array([pose[:3, :3] for pose in filtered_poses])
        t_gripper2base = np.array([pose[:3, 3] for pose in filtered_poses])

        R_handeye, T_handeye = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=rmats_origin,
            t_target2cam=tvecs_origin,
            method=cv2.CALIB_HAND_EYE_DANIILIDIS,
        )
        robot_to_cam = np.eye(4)  # Initialize 4x4 identity matrix
        robot_to_cam[:3, :3] = R_handeye  # Populate rotation
        robot_to_cam[:3, 3] = T_handeye.flatten()  # Populate translation

        # Invert the homogeneous matrix
        cam_to_robot = np.linalg.inv(robot_to_cam)

        # Extract rotation and translation from the inverted matrix
        camera_to_robot_R = cam_to_robot[:3, :3]  # Extract rotation
        camera_to_robot_T = cam_to_robot[:3, 3]  # Extract translation
        logger.info("Hand-eye calibration completed.")

    # save off our work
    calibration_results: CalibrationResults = {
        "camera_matrix_reference": camera_child,
        "dist_coeffs_reference": coeffs_child,
        "dist_coeffs_origin": coeffs_parent,
        "camera_matrix_origin": camera_parent,
        "image_dim_origin": np.array(parent_img_size),
        "image_dim_reference": np.array(child_img_size),
        "R": R,
        "T": T,
        "R_handeye": camera_to_robot_R,
        "T_handeye": camera_to_robot_T,
        "average_reprojection_error": float(np.linalg.norm(err)),
    }
    return calibration_results


def est_camera_t_charuco_board_center(
    img: np.ndarray,
    charuco_board: cv2.aruco_CharucoBoard,
    aruco_dict: cv2.aruco_Dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Localizes the 6D pose of the checkerboard center using Charuco corners with OpenCV's solvePnP.

    The board pose's translation should be at the center of the board, with the orientation in OpenCV format,
    where the +Z points out of the board with the other axes being parallel to the sides of the board.

    Args:
        img (np.ndarray): The input image containing the checkerboard.
        charuco_board (cv2.aruco_CharucoBoard): The Charuco board configuration.
        aruco_dict (cv2.aruco_Dictionary): The Aruco dictionary used to detect markers.
        camera_matrix (np.ndarray): The camera matrix from calibration.
        dist_coeffs (np.ndarray): The distortion coefficients from calibration.

    Returns:
        Optional[Tuple[np.ndarray, np.ndarray]]: The rotation vector and translation vector
        representing the 6D pose of the checkerboard center if found, else None.
    """
    charuco_corners, charuco_ids = detect_charuco_corners(img, charuco_board, aruco_dict)
    if charuco_corners is not None and charuco_ids is not None:
        object_points = get_charuco_board_object_points(charuco_board, charuco_ids)
        image_points = charuco_corners

        # Use solvePnP to estimate the pose of the Charuco board
        success, rvec, tvec = cv2.solvePnP(object_points, np.array(image_points), camera_matrix, dist_coeffs)

        if success:
            # Convert to the camera frame: Adjust the translation vector to be relative to the center of the board
            x_trans = (charuco_board.getSquareLength() * charuco_board.getChessboardSize()[0]) / 2.0
            y_trans = (charuco_board.getSquareLength() * charuco_board.getChessboardSize()[1]) / 2.0
            center_trans = np.array([x_trans, y_trans, 0.0]).reshape((3, 1))
            rmat, _ = cv2.Rodrigues(rvec)

            tvec = tvec + rmat.dot(center_trans)
            tvec_to_camera = tvec
            return np.array(rmat), np.array(tvec_to_camera).ravel()
        else:
            raise ValueError("Pose estimation failed. You likely primed the robot too close to the board.")
    else:
        raise ValueError(
            "Couldn't detect any Charuco boards in the image, "
            "localization failed. Ensure the board is visible from the"
            " primed pose."
        )


def convert_camera_t_viewpoint_to_origin_t_planning_frame(
    origin_t_planning_frame: np.ndarray = np.eye(4),
    planning_frame_t_opencv_camera: np.ndarray = np.eye(4),
    opencv_camera_t_viewpoint: np.ndarray = np.eye(4),
) -> np.ndarray:
    """
    Convert a camera viewpoint given in a camera frame, to the planning frame.

    Camera frame should be in OpenCV/ ROS format, where
    +x should point to the right in the image
    +y should point down in the image
    +z should point into to plane of the image.

    Assuming that origin_t_planning_frame isn't aligned with this format, align it
    so that the viewpoint can be localized as a hand pose command.

    Args:
        origin_t_planning_frame (np.ndarray, optional): 4x4 homogenous transform
            from origin to planning frame. Defaults to np.eye(4).
        planning_frame_t_opencv_camera (np.ndarray, optional): 4x4 homogenous transform
            from planning frame to the camera. Defaults to np.eye(4).
        opencv_camera_t_viewpoint (np.ndarray, optional): 4x4 homogenous transform
            from camera to the future viewpoint. Defaults to np.eye(4).

    Returns:
        np.ndarray: 4x4 homogenous transform of the planning frame of a future viewpoint
            expressed in the origin frame.
    """
    origin_t_opencv_camera_pose = origin_t_planning_frame @ planning_frame_t_opencv_camera
    origin_t_viewpoint = origin_t_opencv_camera_pose @ opencv_camera_t_viewpoint
    origin_t_planning_frame_next = origin_t_viewpoint @ np.linalg.inv(planning_frame_t_opencv_camera)
    return origin_t_planning_frame_next


def get_relative_viewpoints_from_board_pose_and_param(
    R_board: np.ndarray,
    tvec: np.ndarray,
    distances_x: Optional[np.ndarray] = None,
    distances_z: Optional[np.ndarray] = None,
    x_axis_rots: Optional[np.ndarray] = None,
    y_axis_rots: Optional[np.ndarray] = None,
    z_axis_rots: Optional[np.ndarray] = None,
    R_align_board_frame_with_camera: Optional[np.ndarray] = None,
    degree_offset_rotations: bool = True,
) -> List[np.ndarray]:
    """
    Given the position of charuco board, sample viewpoints facing the calibration board for
    the robot's "principal" camera to visit. When the robot moves to these viewpoints,
    it takes synchronized photos with all cameras to use for solving the calibration.

    ROS and OpenCV have the same camera format, where:
        # +x should point to the right in the image
        # +y should point down in the image
        # +z should point into to plane of the image.

    All transforms to cameras are in this format.

    Args:
        R_board (np.ndarray): the rotation of the calibration board in opencv notation relative
            to the "principal" camera.
            (See AutomaticCameraCalibration.localize_target_to_principal_camera)
        tvec (np.ndarray): the translation of the calibration board in opencv notation relative to
            the principal camera
        distances (np.ndarray, optional): What distances
            away from the calibration board's pose (along the Z axis)
            to sample calibration viewpoints from. Defaults to np.arange(0.5, 0.8, 0.1).
        x_axis_rots (np.ndarray, optional): What
            x-axis rotations relative to the camera viewing the board orthogonally
            to apply to sample viewpoints. Defaults to np.arange(-21, 21, 5).
        y_axis_rots (np.ndarray, optional): What
            y-axis rotations relative to the camera viewing the board orthogonally
            to apply to sample viewpoints. Defaults to np.arange(-21, 21, 5).
        z_axis_rots (np.ndarray, optional): What
            z-axis rotations relative to the camera viewing the board orthogonally
            to apply to sample viewpoints. Defaults to np.array([-21, 21, 5]).
        R_align_board_frame_with_camera (np.ndarray, optional): By default,
            the charuco pose rotation Z-axis points out of the pattern,
            and into the camera. However, to sample viewpoints, the camera must face the board,
            so viewpoints can't be sampled directly from the board pose as they would face
            away from the board.
            This rotates the board frame such that the Z axis points away
            from the pattern, so that camera frames facing the board may be sampled
            after applying a distance and offset rotation.
            Defaults to np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).
        degree_offset_rotations (bool, optional): whether to use degree for the
            sampling rotation parameters. Defaults to True.

    Returns:
        List[np.ndarray]: a list of 4x4 homogenous transforms to visit in the "principal" cameras
            frame
    """
    if distances_x is None:
        distances_x = np.arange(0.3, 0.7, 0.1)
    if distances_z is None:
        distances_z = np.arange(-0.2, 0.3, 0.1)
    if x_axis_rots is None:
        x_axis_rots = np.arange(-20, 21, 5)
    if y_axis_rots is None:
        y_axis_rots = np.arange(-20, 21, 5)
    if z_axis_rots is None:
        z_axis_rots = np.arange(-20, 21, 5)
    if R_align_board_frame_with_camera is None:
        R_align_board_frame_with_camera = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

    if degree_offset_rotations:
        x_axis_rots = [radians(deg) for deg in x_axis_rots]
        y_axis_rots = [radians(deg) for deg in y_axis_rots]
        z_axis_rots = [radians(deg) for deg in z_axis_rots]

    translations = [(tvec + R_board[:, 2] * dist + R_board[:, 0] * d2) for dist in distances_z for d2 in distances_x]
    R_base = R_board @ R_align_board_frame_with_camera

    def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Convert Euler angles to a rotation matrix using OpenCV, for the sake
        of CLI convenience, conciseness, while not depending on an external transform lib.
        """
        R_x = cv2.Rodrigues(np.array([roll, 0, 0]))[0]
        R_y = cv2.Rodrigues(np.array([0, pitch, 0]))[0]
        R_z = cv2.Rodrigues(np.array([0, 0, yaw]))[0]
        return R_z @ R_y @ R_x

    poses = []
    for t in translations:
        for x_axis_rot in x_axis_rots:
            for y_axis_rot in y_axis_rots:
                for z_axis_rot in z_axis_rots:
                    # Get the rotation matrix from Euler angles
                    R_rot = euler_to_rotation_matrix(x_axis_rot, y_axis_rot, z_axis_rot)

                    # Combine with the base rotation
                    R_base_modified = R_base @ R_rot

                    transform = np.eye(4)
                    transform[:-1, -1] = t.reshape((3,))
                    transform[:-1, :-1] = R_base_modified
                    poses.append(transform)

    logger.info(f"Calculated {len(poses)} relative viewpoints to visit relative to the target.")
    return poses


def create_ideal_charuco_image(charuco_board: cv2.aruco_CharucoBoard, dim=(500, 700), colorful=False):
    if OPENCV_VERSION < OPENCV_CHARUCO_LIBRARY_CHANGE_VERSION:
        img = charuco_board.draw(outSize=dim)
    else:
        img = charuco_board.generateImage(outSize=dim)
    if colorful:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        return img


def charuco_pose_sanity_check(
    img: np.ndarray, charuco_board: cv2.aruco_CharucoBoard, aruco_dict: cv2.aruco_Dictionary
) -> np.ndarray:
    """
    Visualize the Charuco board pose detection on an image for sanity-checking.

    Uses an identity camera matrix and zero distortion to estimate and display
    the 6D pose of the board center, overlaying 3D axes on the image.

    Args:
        img (np.ndarray): The input image containing the Charuco board.
        charuco_board (cv2.aruco_CharucoBoard): The Charuco board configuration.
        aruco_dict (cv2.aruco_Dictionary): The Aruco dictionary used to detect markers.

    Returns:
        img_with_axes (np.ndarray): The image with the pose axes drawn.
    """

    def is_z_axis_out_of_board(tvec: np.ndarray) -> bool:
        """Determine if the Z-axis points out of the Charuco board (towards the camera)."""
        return tvec[2] > 0

    def visualize_pose_with_axis(
        img: np.ndarray,
        rmat: np.ndarray,
        tvec: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        axis_length: float = 0.115,
    ) -> np.ndarray:
        """Draws the 3D pose axes on the image and displays whether the Z-axis is out or into the board."""
        axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length], [0, 0, 0]]).reshape(-1, 3)
        imgpts, _ = cv2.projectPoints(axis, rmat, tvec, camera_matrix, dist_coeffs)
        z_out_of_board = is_z_axis_out_of_board(tvec)
        img_with_axes = img.copy()
        origin = tuple(imgpts[3].ravel().astype(int))
        img_with_axes = cv2.line(img_with_axes, origin, tuple(imgpts[0].ravel().astype(int)), (0, 0, 255), 3)  # X
        img_with_axes = cv2.line(img_with_axes, origin, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 3)  # Y
        img_with_axes = cv2.line(img_with_axes, origin, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 3)  # Z
        if not z_out_of_board:
            logger.warning("This tool assumes that Z axis is out of board, but it was detected as into board.")
        window_name = f'Charuco Board Pose ({"Z-axis out of board" if z_out_of_board else "Z-axis into board"})'
        cv2.imshow(window_name, img_with_axes)
        elapsed_time = 0
        wait_interval = 100
        max_wait_time = 20
        while elapsed_time < max_wait_time * 1000:
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
            key = cv2.waitKey(wait_interval)
            if key != -1:
                break
            elapsed_time += wait_interval
        cv2.destroyAllWindows()
        return img_with_axes

    camera_matrix = np.eye(3, dtype=np.float32)
    dist_coeffs = np.zeros(5, dtype=np.float32)
    rmat, tvec = est_camera_t_charuco_board_center(img, charuco_board, aruco_dict, camera_matrix, dist_coeffs)
    return visualize_pose_with_axis(img, rmat, tvec, camera_matrix, dist_coeffs)
