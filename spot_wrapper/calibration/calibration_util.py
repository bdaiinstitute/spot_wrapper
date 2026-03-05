# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import argparse
import logging
import os
import re
from datetime import datetime
from glob import glob
from time import sleep
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import yaml

from spot_wrapper.calibration.calibration_helpers import (
    AutomaticCameraCalibrationRobot,
)
from spot_wrapper.calibration.charuco_board_detection import (
    create_ideal_charuco_image,
    detect_charuco_corners,
    get_relative_viewpoints_from_board_pose_and_param,
    multistereo_calibration_charuco,
)

logger = logging.getLogger(__name__)

directories = ["parent", "child", "poses", "depth"]


def load_dataset_from_path(path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Load image dataset from path in a way that's compatible with multistereo_calibration_charuco.

    Also, load the poses if they are available.

    See Using the CLI Tool To Calibrate On an Existing Dataset section in the README
    to see the expected folder/data structure for this method to work

    Args:
        path (str): The parent path

    Raises:
        ValueError: Not possible to load the images

    Returns:
        np.ndarray: The image dataset
    """

    def alpha_numeric(x):
        return re.search("(\\d+)(?=\\D*$)", x).group() if re.search("(\\d+)(?=\\D*$)", x) else x

    # List all directories within the given path and sort them
    dirs = [d for d in os.listdir(path) if os.path.isdir(os.path.join(path, d))]
    if len(dirs) == 0:
        raise ValueError("No sub-dirs found in datapath from which to load images.")
    dirs = sorted(dirs, key=alpha_numeric)  # Assuming dir names are integers like "0", "1", etc.

    # Initialize an empty list to store images
    images = []
    poses = None

    for dir_name in dirs:
        path_match = os.path.join(path, dir_name, "*")
        files = sorted(
            glob(path_match),
            key=alpha_numeric,
        )
        if dir_name != "poses":
            # Read images and store them
            images.append([cv2.imread(fn) for fn in files])
        else:
            poses = np.array([np.load(fn) for fn in files])

    # Convert the list of lists into a NumPy array
    # The array shape will be (number_of_images, number_of_directories)
    images = np.array(images, dtype=object)
    # Transpose the array so that you can access it as images[:, axis]
    images = np.transpose(images, (1, 0))

    return images, poses


def save_calibration_parameters(
    data: Dict,
    output_path: str,
    num_images: int,
    tag: str,
    parent_frame: str,
    child_frame: str,
    parser_args: Optional[argparse.Namespace] = None,
    unsafe: bool = False,
) -> Dict:
    """
    Dump the results of a calibration, and the metadata associated with the command that
    created it, to a file. 0 is RGB and 1 is depth.

    Args:
        data (Dict): The results of the calibration
        output_path (str): The path/name of what to create
        num_images (int): How many images were used for this calibration
        tag (str): What tag to give as the heading/name for this calibration
        parser_args (Optional[argparse.Namespace], optional): The args that were
            used to create the calibration. Defaults to None.
        unsafe (bool, optional): Whether to overwrite existing calibrations of the same name,
            and to ignore recommended naming scheme. Defaults to False.
    """

    def flatten_matrix(matrix: np.ndarray) -> List:
        return matrix.flatten().tolist()

    def process_data_with_nested_dictionaries(
        data: Dict,
    ) -> Tuple[Dict, Dict]:
        cameras: Dict[int, Dict[str, List]] = {}
        relations: Dict[int, Dict[int, Dict[str, List]]] = {}

        for value in iter(data.values()):
            origin_cam = 0
            reference_cam = 1

            # Process origin camera data
            if origin_cam not in cameras:
                cameras[origin_cam] = {
                    "camera_matrix": flatten_matrix(value["camera_matrix_origin"]),
                    "dist_coeffs": flatten_matrix(value["dist_coeffs_origin"]),
                    "image_dim": flatten_matrix(value["image_dim_origin"]),
                }

            # Process reference camera data
            if reference_cam not in cameras:
                cameras[reference_cam] = {
                    "camera_matrix": flatten_matrix(value["camera_matrix_reference"]),
                    "dist_coeffs": flatten_matrix(value["dist_coeffs_reference"]),
                    "image_dim": flatten_matrix(value["image_dim_reference"]),
                }

            # Store the stereo calibration rotation and translation
            if origin_cam not in relations:
                relations[origin_cam] = {}
            relations[origin_cam][reference_cam] = {
                "R": flatten_matrix(value["R"]),
                "T": flatten_matrix(value["T"]),
            }

            # Now add R_handeye and T_handeye if they exist in the data
            if "R_handeye" in value and "T_handeye" in value:
                relations[origin_cam]["planning_frame"] = {
                    "R": flatten_matrix(value["R_handeye"]),
                    "T": flatten_matrix(value["T_handeye"]),
                }

        return cameras, relations

    # Handle empty or missing tag
    if not tag:
        tag = "default"

    # Load existing YAML file if it exists
    output_path = os.path.abspath(os.path.expanduser(output_path))
    if os.path.exists(output_path):
        with open(output_path, "r") as file:
            existing_data = yaml.safe_load(file) or {}
    else:
        existing_data = {}

    # Check for overwriting existing tag
    if not unsafe:
        overwrite_confirmed = False
        while not overwrite_confirmed:
            if tag in existing_data:
                sure = input(f"Tag '{tag}' already exists. Overwrite existing calibration (y/n)? ").strip().lower()
                if sure == "y":
                    logger.warning(f"Overwriting the existing tag '{tag}' in {output_path}.")
                    overwrite_confirmed = True
                elif sure == "n":
                    tag = input("Enter a new tag: ").strip() or "default"
                    if tag not in existing_data:
                        overwrite_confirmed = True
            elif not existing_data and tag != "default":
                sure = (
                    input(
                        "The file is empty. It is recommended to use "
                        "the default tag. Are you sure you want to use"
                        f" '{tag}' instead? (y/n): "
                    )
                    .strip()
                    .lower()
                )
                if sure == "y":
                    overwrite_confirmed = True
                elif sure == "n":
                    tag = "default"
                    overwrite_confirmed = True
            else:
                overwrite_confirmed = True

    # Process the new calibration data
    cameras, relations = process_data_with_nested_dictionaries(data)

    first_result = next(iter(data.values()))
    xform = np.eye(4)
    if "R_handeye" in first_result and "T_handeye" in first_result:
        xform[:3, :3] = first_result["R_handeye"]
        xform[:3, 3] = np.array(first_result["T_handeye"]).flatten()
    else:
        xform[:3, :3] = first_result["R"]
        xform[:3, 3] = first_result["T"].flatten()

    # Prepare the output data under the specified tag
    run_params: Dict[str, Any] = {}
    run_params["num_images"] = num_images
    run_params["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Include parser parameters, excluding 'password' and 'username'
    if parser_args is not None:
        for arg in vars(parser_args):
            if arg not in ["password", "username", "result_path"]:
                run_params[arg] = getattr(parser_args, arg)
    else:
        logger.warning("Saving calibration, but not the parameters used to obtain it.")

    # Convert any tuples in run_params (like stereo_pairs) to lists
    if "stereo_pairs" in run_params:
        run_params["stereo_pairs"] = [list(pair) for pair in run_params["stereo_pairs"]]

    # Save the updated data under the specified tag
    tagged_data = {
        "intrinsic": cameras,
        "extrinsic": relations,
        "run_params": run_params,
        f"{child_frame}_t_{parent_frame}": flatten_matrix(xform),
    }
    existing_data[tag] = tagged_data

    # Ensure the directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # Save to YAML file
    with open(output_path, "w") as file:
        yaml.dump(
            existing_data,
            file,
            default_flow_style=None,
            sort_keys=False,
        )
    logger.info(f"Saved calibration to file {output_path} under tag '{tag}'")
    return existing_data


def get_multiple_perspective_camera_calibration_dataset(
    auto_cam_cal_robot: AutomaticCameraCalibrationRobot,
    max_num_images: int = 10000,
    distances_x: Optional[np.ndarray] = None,
    distances_z: Optional[np.ndarray] = None,
    x_axis_rots: Optional[np.ndarray] = None,
    y_axis_rots: Optional[np.ndarray] = None,
    z_axis_rots: Optional[np.ndarray] = None,
    use_degrees: bool = True,
    settle_time: float = 0.1,
    data_path: str = os.path.expanduser("~"),
    save_data: Optional[bool] = True,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Move the robot to multiple viewpoints and capture time-synchronized images from all cameras
    at each viewpoint for use in calibration.

    Args:
        auto_cam_cal_robot (AutomaticCameraCalibrationRobot): The robot to automatically calibrate.
        max_num_images (int, optional): Maximum number of captures before cutting off the
            viewpoint sweep early. Defaults to 10000.
        distances_x (np.ndarray, optional): X-axis distances from the board to sample.
            Defaults to None (uses charuco_board_detection defaults).
        distances_z (np.ndarray, optional): Z-axis distances from the board to sample.
            Defaults to None (uses charuco_board_detection defaults).
        x_axis_rots (np.ndarray, optional): X-axis rotations to sample. Defaults to None.
        y_axis_rots (np.ndarray, optional): Y-axis rotations to sample. Defaults to None.
        z_axis_rots (np.ndarray, optional): Z-axis rotations to sample. Defaults to None.
        use_degrees (bool, optional): Whether rotation parameters are in degrees. Defaults to True.
        settle_time (float, optional): Seconds to wait after moving before capturing.
            Defaults to 0.1.
        data_path (str, optional): Directory to save the dataset if save_data is True.
            Defaults to the user home directory.
        save_data (Optional[bool], optional): Whether to save captured images and poses.
            Defaults to True.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Array of captured image sets and array of robot poses.
    """
    primed_pose = auto_cam_cal_robot.move_cameras_to_see_calibration_target()
    logger.info("Primed arm...")
    sleep(settle_time)
    images = auto_cam_cal_robot.capture_images()

    R_vision_to_target, tvec_vision_to_target = auto_cam_cal_robot.localize_target_to_principal_camera(images)
    viewpoints = get_relative_viewpoints_from_board_pose_and_param(
        R_vision_to_target,
        tvec_vision_to_target,
        distances_x=distances_x,
        distances_z=distances_z,
        x_axis_rots=x_axis_rots,
        y_axis_rots=y_axis_rots,
        z_axis_rots=z_axis_rots,
        degree_offset_rotations=use_degrees,
    )
    calibration_images = []
    logger.info("Beginning Calibration")
    idx = 0
    poses = []
    while idx < max_num_images and idx < len(viewpoints):
        logger.info(f"Visiting viewpoint {idx + 1} of {min(len(viewpoints), max_num_images)}")
        viewpoint = viewpoints[idx]
        _initial_pose, new_pose = auto_cam_cal_robot.offset_cameras_from_current_view(
            transform_offset=viewpoint,
            origin_t_planning_frame=primed_pose,
            duration_sec=0.1,
        )
        poses.append(new_pose)
        logger.info("At viewpoint, waiting to settle")
        sleep(settle_time)
        images = auto_cam_cal_robot.capture_images()
        logger.info("Snapped pics ;)")
        calibration_images.append(images)
        idx = len(calibration_images)
        if save_data:
            if idx == 1:
                # Create numerical camera-index folders
                for jdx in range(len(images)):
                    os.makedirs(os.path.join(data_path, str(jdx)), exist_ok=True)
                os.makedirs(os.path.join(data_path, "poses"), exist_ok=True)
            logger.info(f"Saving image batch {idx}")
            for jdx, image in enumerate(images):
                cv2.imwrite(
                    os.path.join(data_path, str(jdx), f"{idx}.png"),
                    image,
                )
            np.save(os.path.join(data_path, "poses", f"{idx}.npy"), new_pose)
    return (np.array(calibration_images, dtype=object), poses)


def calibration_helper(
    images: Union[List[np.ndarray], np.ndarray],
    args: argparse.Namespace,
    charuco: cv2.aruco_CharucoBoard,
    aruco_dict: cv2.aruco_Dictionary,
    poses: np.ndarray,
    result_path: str = None,
    parent_frame: str = "body",
    child_frame: str = "camera",
) -> dict:
    logger.warning(
        f"Calibrating from {len(images)} images.. for every "
        f"{args.photo_utilization_ratio} recorded photos 1 is used to calibrate"
    )
    if not args.allow_default_internal_corner_ordering:
        logger.warning("Turning off corner swap (needed for localization) for calibration solution...")
        logger.warning("Corner swap needed for initial localization, but breaks calibration.")
        logger.warning("See https://github.com/opencv/opencv/issues/26126")
        detect_charuco_corners(
            create_ideal_charuco_image(charuco_board=charuco),
            charuco_board=charuco,
            aruco_dict=aruco_dict,
            enforce_ascending_ids_from_bottom_left_corner=False,
        )
    calibration = multistereo_calibration_charuco(
        images[:: args.photo_utilization_ratio],
        desired_stereo_pairs=args.stereo_pairs,
        charuco_board=charuco,
        aruco_dict=aruco_dict,
        poses=poses,
    )
    logger.info(f"Finished script, obtained {calibration}")
    logger.info("Saving calibration param...")

    if result_path is None:
        result_path = "calibration_result.yaml"  # default file name if none provided

    args.result_path = result_path

    # Save the calibration parameters if a valid result path is provided
    calibration_dict = save_calibration_parameters(
        data=calibration,
        output_path=args.result_path,
        num_images=len(images[:: args.photo_utilization_ratio]),
        tag=args.tag,
        parent_frame=parent_frame,
        child_frame=child_frame,
        parser_args=args,
        unsafe=args.unsafe_tag_save,
    )
    return calibration_dict
