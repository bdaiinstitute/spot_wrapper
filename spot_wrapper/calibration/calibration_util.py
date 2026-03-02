# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# Copyreference (c) 2024 Robotics and AI Institute LLC dba RAI Institute. All references reserved.

import argparse
import logging
import os
import re
from datetime import datetime
from glob import glob
from pathlib import Path
from time import sleep
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge

# from message_filters import ApproximateTimeSynchronizer, Subscriber
# from rclpy.callback_groups import CallbackGroup
# from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as RosImage

from spot_wrapper.calibration.automatic_camera_calibration_robot import (
    AutomaticCameraCalibrationRobot,
)
from spot_wrapper.calibration.calibration_helpers import (
    # CalibrationResults,
    CameraIntrinsics,
    Image,
)

# TopicMsgPair,
from spot_wrapper.calibration.charuco_board_detection import (
    create_ideal_charuco_image,
    detect_charuco_corners,
    get_relative_viewpoints_from_board_pose_and_param,
    multistereo_calibration_charuco,
)

logger = logging.getLogger(__name__)

directories = ["parent", "child", "poses", "depth"]


def camera_info_to_dict(camera_info: CameraInfo, camera_name: str) -> dict[str, Any]:
    return {
        "image_width": camera_info.width,
        "image_height": camera_info.height,
        "camera_name": camera_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": camera_info.k},
        "distortion_model": camera_info.distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(camera_info.d), "data": camera_info.d},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": camera_info.r},
        "projection_matrix": {"rows": 3, "cols": 4, "data": camera_info.p},
        "binning_x": camera_info.binning_x,
        "binning_y": camera_info.binning_y,
        "roi": {
            "x_offset": camera_info.roi.x_offset,
            "y_offset": camera_info.roi.y_offset,
            "height": camera_info.roi.height,
            "width": camera_info.roi.width,
            "do_rectify": camera_info.roi.do_rectify,
        },
    }


def save_CameraInfo_2_file(msg: CameraInfo, camera_name: str, file_path: Path) -> None:
    """Saves a CameraInfo message to a YAML file."""
    cam_info_msg_dict = camera_info_to_dict(camera_info=msg, camera_name=camera_name)

    with open(file_path, "w") as f:
        yaml.dump(cam_info_msg_dict, f, default_flow_style=False)


def load_CameraInfo_from_file(file_path: Path) -> CameraInfo:
    """Loads a CameraInfo message from a YAML file."""
    logging.info(f"Loading CameraInfo from {file_path}")
    with open(file_path, "r") as f:
        cam_info_msg_dict = yaml.unsafe_load(f)

    cam_info_msg = CameraInfo()
    cam_info_msg.width = cam_info_msg_dict["image_width"]
    cam_info_msg.height = cam_info_msg_dict["image_height"]
    cam_info_msg.header.frame_id = cam_info_msg_dict["camera_name"]
    cam_info_msg.k = cam_info_msg_dict["camera_matrix"]["data"]
    cam_info_msg.distortion_model = cam_info_msg_dict["distortion_model"]
    cam_info_msg.d = cam_info_msg_dict["distortion_coefficients"]["data"]
    cam_info_msg.r = cam_info_msg_dict["rectification_matrix"]["data"]
    cam_info_msg.p = cam_info_msg_dict["projection_matrix"]["data"]
    cam_info_msg.binning_x = cam_info_msg_dict["binning_x"]
    cam_info_msg.binning_y = cam_info_msg_dict["binning_y"]
    roi_dict = cam_info_msg_dict["roi"]
    cam_info_msg.roi.x_offset = roi_dict["x_offset"]
    cam_info_msg.roi.y_offset = roi_dict["y_offset"]
    cam_info_msg.roi.height = roi_dict["height"]
    cam_info_msg.roi.width = roi_dict["width"]
    cam_info_msg.roi.do_rectify = roi_dict["do_rectify"]

    return cam_info_msg


def load_images_from_path(path: Path) -> Dict[str, Dict[str, np.ndarray]]:
    """
    Load images dataset from path in a way that's compatible with multistereo_calibration_charuco.

    Also, load the poses if they are available.

    See Using the CLI Tool To Calibrate On an Existing Dataset section in the README
    to see the expected folder/data structure for this method to work

    Args:
        path (str): The parent path

    Raises:
        ValueError: Not possible to load the images

    Returns:
        dict[str, np.ndarray]: The image dataset
    """

    def alpha_numeric(x: str) -> Any:
        matcha = re.search("(\\d+)(?=\\D*$)", x)
        # \\d+ Matches one or more digits (0-9),
        # \\D* Matches zero or more non-digit characters,
        # $ asserts position at the end of the string.
        if matcha:
            return int(matcha.group())
        return x

    def load_images_from_dir(path: Path) -> Dict[str, np.ndarray]:
        print(f"-----------------------Loading images from {path}")
        files = sorted(
            glob(os.path.join(path, "*.png")),
            key=alpha_numeric,
        )
        try:
            return {
                Path(fn).name: cv2.imread(fn, cv2.IMREAD_GRAYSCALE).astype(np.uint8)
                for fn in files
                if fn.lower().endswith(".png")
            }
        except Exception as e:
            logging.error(f"Error loading images from {files}: {e}")
            return {}

    # Initialize an empty dict to store images
    images = dict()

    # directories we care about here
    parent_path = os.path.join(path, "parent")
    child_path = os.path.join(path, "child")

    # load images from both directories
    images["parent"] = load_images_from_dir(Path(parent_path))
    images["child"] = load_images_from_dir(Path(child_path))

    return images


# TODO
# def load_calibration_parameters(input_path: Path) -> CalibrationResults:
#     """
#     Load calibration parameters from a YAML file.

#     Args:
#         input_path (Path): The path to the YAML file containing calibration parameters.
#     Returns:
#         CalibrationResults: The loaded calibration parameters.
#     Throws:
#         FileNotFoundError: If the specified file does not exist.
#         KeyError: If required keys are missing in the YAML file.
#     """
#     with open(input_path, "r") as file:
#         calib_data = yaml.safe_load(file)

#     parent_camera = np.array(calib_data["default"]["intrinsic"][0]["camera_matrix"]).reshape((3, 3))
#     parent_dist_coeffs = np.array(calib_data["default"]["intrinsic"][0]["dist_coeffs"]).reshape((-1, 1))
#     parent_image_dim = np.array(calib_data["default"]["intrinsic"][0]["image_dim"])
#     child_camera = np.array(calib_data["default"]["intrinsic"][1]["camera_matrix"]).reshape((3, 3))
#     child_dist_coeffs = np.array(calib_data["default"]["intrinsic"][1]["dist_coeffs"]).reshape((-1, 1))
#     child_image_dim = np.array(calib_data["default"]["intrinsic"][1]["image_dim"])
#     R = np.array(calib_data["default"]["extrinsic"][0][1]["R"]).reshape((3, 3))
#     T = np.array(calib_data["default"]["extrinsic"][0][1]["T"]).reshape((-1, 3))

#     # saving out reproj err not supported, currently.
#     # does not save out reproj err.
#     # So we set it to 0 here.
#     calib_results: CalibrationResults = {
#         "camera_matrix_origin": parent_camera,
#         "dist_coeffs_origin": parent_dist_coeffs,
#         "image_dim_origin": parent_image_dim,
#         "camera_matrix_reference": child_camera,
#         "dist_coeffs_reference": child_dist_coeffs,
#         "image_dim_reference": child_image_dim,
#         "R": R,
#         "T": T,
#         "R_handeye": np.eye(3),
#         "T_handeye": np.zeros((3, 1)),
#         "average_reprojection_error": 0,
#     }

#     return calib_results


def load_dataset_from_path(pathdir: Path) -> Tuple[Dict[str, Dict[str, np.ndarray]], CameraInfo, CameraInfo]:
    """
    load the data for images, hand_cam_info, ext_cam_info

    Args:
        pathdir (Path): The absolute pathname to directory containing the dataset.

    Returns:
        Tuple[np.ndarray, CameraInfo, CameraInfo]: The loaded images, hand camera info, and external camera info.
    """
    images = load_images_from_path(pathdir)
    hciyaml = os.path.join(pathdir, Path("parent"), Path("camera_info.yaml"))
    eciyaml = os.path.join(pathdir, Path("child"), Path("camera_info.yaml"))
    hand_cam_info = load_CameraInfo_from_file(Path(hciyaml))
    ext_cam_info = load_CameraInfo_from_file(Path(eciyaml))

    return images, hand_cam_info, ext_cam_info


def create_calibration_save_folders(path: Path) -> None:
    """
    Create a folder hierarchy to record a calibration

    Args:
        path (Path): The parent path

    Raises:
        ValueError: Not possible to create the folders, or no path specified
    """
    if path is None:
        raise ValueError("No path to save to. you can do better than this.")
    else:
        for folder in directories:
            cam_path = os.path.join(path, folder)

            logger.info(f"Creating image folder at {cam_path}")
            os.makedirs(cam_path, exist_ok=True)
        os.makedirs(os.path.join(path, "poses"), exist_ok=True)
        logger.info("Done creating folders.")


# TODO
# def save_dataset_to_dir(
#     path: Path, images_dict: dict[str, list[np.ndarray]], camera_info_dict: dict[str, CameraInfo]
# ) -> None:
#     """
#     Save image dataset to path in a way that's compatible with multistereo_calibration_charuco.

#     Also, save the camera infos.

#     See Using the CLI Tool To Calibrate On an Existing Dataset section in the README
#     to see the expected folder/data structure for this method to work

#     Args:
#         path (str): The parent path
#         images_dict (dict[int, list[np.ndarray]]): The image dataset by camera index
#         camera_info_dict (dict[int, CameraInfo]): The camera info by camera index
#     """

#     create_calibration_save_folders(path)

#     for cam_idx, images in images_dict.items():
#         cam_dir = path / Path(str(cam_idx))
#         for img_idx, img in enumerate(images):
#             img_path = cam_dir / Path(f"{img_idx}.png")
#             cv2.imwrite(str(img_path), img)
#             # np.save(cam_dir / Path("camera_info.npy"), camera_info_dict[cam_idx])
#             save_CameraInfo_2_file(camera_info_dict[cam_idx], str(cam_idx), cam_dir / Path("camera_info.yaml"))


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
            """ if "R_handeye" in value and "T_handeye" in value:
                relations[origin_cam]["planning_frame"] = {
                    "R": flatten_matrix(value["R_handeye"]),
                    "T": flatten_matrix(value["T_handeye"]),
                } """

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

    xform = np.eye(4)
    xform[:3, :3] = next(iter(data.values()))["R"]
    xform[:3, 3] = next(iter(data.values()))["T"].flatten()

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


def camera_info_to_intrinsics(msg: CameraInfo) -> CameraIntrinsics:
    """Construct a `CameraIntrinsics` instance from a `CameraInfo` message.

    Args:
        msg: The message to convert. The fields that are copied are the intrinsics (`k`), the distortion (`d`), width,
             and height. All other fields are ignored.

    Returns:
        The converted camera intrinsics object.

    Raises:
        ValueError if `k` contains non-zero skew values.
    """
    camera_matrix = np.array(msg.k).reshape((3, 3))
    distortion_coeffs = np.array(msg.d)
    return CameraIntrinsics(
        height=msg.height, width=msg.width, camera_matrix=camera_matrix, distortion_coeffs=distortion_coeffs
    )


def ros_image_to_image(ros_image: RosImage, cv_bridge: CvBridge | None = None, ros_encoding: str = "rgb8") -> Image:
    """
    Converts from ros image to our generic image datatype
    """
    if cv_bridge is None:
        cv_bridge = CvBridge()

    img = cv_bridge.imgmsg_to_cv2(ros_image, desired_encoding=ros_encoding)

    return Image.from_numpy(img)


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
        result_path = input("Please provide a path to save the calibration results (or type 'No' to skip): ")

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


# def create_time_synchronizer(
#     node: Node,
#     topic_msg_type_pairs: Sequence[TopicMsgPair],
#     callback: Callable[..., None],
#     callback_group: Optional[CallbackGroup] = None,
#     queue_size: int = 30,
#     slop_sec: float = 0.3,
# ) -> ApproximateTimeSynchronizer:
#     """Creates an `ApproximateTimeSynchronizer` for a list of topic names and msg types

#     See `$BDAI/projects/watch_understand_do/ws/src/wud_ros/wud_ros/look_at_that/lang_to_pcd_server.py` for an example
#     Also see: https://github.com/ros2/message_filters/blob/humble/src/message_filters/__init__.py#L242
#     """
#     subscribers = [
#         Subscriber(node, msg_type, topic_name, qos_profile=qos_profile, callback_group=callback_group)
#         for topic_name, msg_type, qos_profile in topic_msg_type_pairs
#     ]
#     time_synchronizer = ApproximateTimeSynchronizer(subscribers, queue_size, slop_sec)
#     time_synchronizer.registerCallback(callback)

#     return time_synchronizer
