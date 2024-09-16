# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

import argparse
import logging
from typing import Tuple

import cv2
import numpy as np

from spot_wrapper.calibration.automatic_camera_calibration_robot import AutomaticCameraCalibrationRobot
from spot_wrapper.calibration.calibrate_multistereo_cameras_with_charuco_cli import (
    calibration_helper,
    calibrator_cli,
    setup_calibration_param,
)
from spot_wrapper.calibration.calibration_util import (
    get_multiple_perspective_camera_calibration_dataset,
    load_dataset_from_path,
)
from spot_wrapper.calibration.spot_in_hand_camera_calibration import (
    SpotInHandCalibration,
)

logging.basicConfig(
    level=logging.INFO,
)

logger = logging.getLogger(__name__)


def create_robot(
    args: argparse.ArgumentParser, charuco: cv2.aruco_CharucoBoard, aruco_dict: cv2.aruco_Dictionary
) -> Tuple[AutomaticCameraCalibrationRobot, argparse.Namespace]:
    # Replace with your AutomaticCameraCalibrationRobot
    in_hand_bot = SpotInHandCalibration(args.ip, args.username, args.password)
    in_hand_bot._set_localization_param(
        charuco_board=charuco,
        aruco_dict=aruco_dict,
        resolution=(
            args.spot_rgb_photo_width,
            args.spot_rgb_photo_height,
        ),
    )
    try:
        args.robot_name = in_hand_bot.robot.get_cached_robot_id().nickname
    except Exception:
        logger.warning("Could not determine cached robot nickname, saving name as unknown")
        args.robot_name = "unknown"
    return in_hand_bot, args


def create_robot_parser() -> argparse.ArgumentParser:
    parser = calibrate_robot_cli()
    return spot_cli(parser=parser)  # Replace with robot specific parsing


def spot_main() -> None:
    parser = create_robot_parser()
    args, aruco_dict, charuco = setup_calibration_param(parser)

    if not args.from_data:
        logger.warning("This script moves the robot around. !!! USE AT YOUR OWN RISK !!!")
        logger.warning("HOLD Ctrl + C NOW TO CANCEL")
        logger.warning("The calibration board should be about a meter away with nothing within a meter of the robot.")
        logger.warning("The robot should NOT be docked, and nobody should have robot control")
        # sleep(5)

        in_hand_bot, args = create_robot(args, charuco=charuco, aruco_dict=aruco_dict)

        images, poses = get_multiple_perspective_camera_calibration_dataset(
            auto_cam_cal_robot=in_hand_bot,
            max_num_images=args.max_num_images,
            distances=np.arange(*args.dist_from_board_viewpoint_range),
            x_axis_rots=np.arange(*args.x_axis_rot_viewpoint_range),
            y_axis_rots=np.arange(*args.y_axis_rot_viewpoint_range),
            z_axis_rots=np.arange(*args.z_axis_rot_viewpoint_range),
            use_degrees=args.use_degrees,
            settle_time=args.settle_time,
            data_path=args.data_path,
            save_data=args.save_data,
        )
    else:
        logger.info(f"Loading images from {args.data_path}")
        images, poses = load_dataset_from_path(args.data_path)

    calibration_helper(images=images, args=args, charuco=charuco, aruco_dict=aruco_dict, poses=poses)


def calibrate_robot_cli(parser: argparse.ArgumentParser = None) -> argparse.ArgumentParser:
    if parser is None:
        parser = calibrator_cli()

    parser.add_argument(
        "--dist_from_board_viewpoint_range",
        "-dfbvr",
        nargs="+",
        type=float,
        dest="dist_from_board_viewpoint_range",
        default=[0.5, 0.6, 0.1],
        help=(
            "What distances to conduct calibrations at relative to the board. (along the normal vector) "
            "Three value array arg defines the [Start, Stop), step. for the viewpoint sweep. "
            "These are used to sample viewpoints for automatic collection. "
        ),
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--degrees",
        "-d",
        dest="use_degrees",
        action="store_true",
        default=True,
        help="Use degrees for rotation ranges (default)",
    )
    group.add_argument(
        "--radians",
        "-r",
        dest="use_degrees",
        action="store_false",
        help="Use radians for rotation ranges",
    )
    defaults = [[-10, 11, 10], [-10, 11, 10], [-10, 11, 10]]
    for idx, axis in enumerate(["x", "y", "z"]):
        parser.add_argument(
            f"--{axis}_axis_rot_viewpoint_range",
            f"-{axis}arvr",
            nargs="+",
            type=float,
            default=defaults[idx],
            dest=f"{axis}_axis_rot_viewpoint_range",
            help=(
                f"What range of viewpoints around {axis}-axis to sample relative to boards normal vector. "
                "Three value array arg defines the [Start, Stop), step. for the viewpoint sweep "
                "These are used to sample viewpoints for automatic collection. "
                "Assuming that the camera pose is in opencv/ROS format. "
            ),
        )

    parser.add_argument(
        "--max_num_images",
        dest="max_num_images",
        type=int,
        default=10000,
        help="The maximum number of images",
        required=False,
    )

    parser.add_argument(
        "--settle_time",
        "-st",
        dest="settle_time",
        type=float,
        default=1.0,
        help="How long to wait after movement to take a picture; don't want motion blur",
    )

    # path things
    parser.add_argument(
        "--save_data",
        "-sd",
        dest="save_data",
        action="store_true",
        help="whether to save the images to file",
    )

    parser.add_argument(
        "--from_data",
        "-fd",
        dest="from_data",
        action="store_true",
        help="Whether to only calibrate from recorded dataset on file.",
    )

    return parser


def spot_cli(parser: argparse.ArgumentParser = None) -> argparse.ArgumentParser:
    if parser is None:
        parser = calibrate_robot_cli()

    parser.add_argument(
        "--ip",
        "-i",
        "-ip",
        dest="ip",
        type=str,
        help="The IP address of the Robot to calibrate",
        required=True,
    )
    parser.add_argument(
        "--user",
        "-u",
        "--username",
        dest="username",
        type=str,
        help="Robot Username",
        required=True,
    )
    parser.add_argument(
        "--pass",
        "-pw",
        "--password",
        dest="password",
        type=str,
        help="Robot Password",
        required=True,
    )

    parser.add_argument(
        "--spot_rgb_photo_width",
        "-dpw",
        type=int,
        default=640,
        dest="spot_rgb_photo_width",
        help="What resolution use for Spot's RGB Hand Camera (width). Currently, only 640 and 1920 are supported",
    )

    parser.add_argument(
        "--spot_rgb_photo_height",
        "-dph",
        type=int,
        default=480,
        help="What resolution use for Spot's RGB Hand Camera (width). Currently, only 480 and 1080 are supported",
    )
    return parser


if __name__ == "__main__":
    spot_main()
