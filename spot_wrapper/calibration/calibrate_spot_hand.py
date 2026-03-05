# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

import argparse
import logging
from typing import Tuple

import cv2
import numpy as np
import yaml

from spot_wrapper.calibration.calibration_clis import (
    calibrate_robot_cli,
    setup_calibration_param,
    spot_cli,
)
from spot_wrapper.calibration.calibration_helpers import AutomaticCameraCalibrationRobot
from spot_wrapper.calibration.calibration_util import (
    calibration_helper,
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


def _require_robot_credentials(args: argparse.Namespace) -> None:
    """Raise a clear error when robot credentials are needed but were not supplied."""
    missing = [
        flag for flag, val in (("--ip", args.ip), ("--user", args.username), ("--pass", args.password)) if not val
    ]
    if missing:
        raise ValueError(f"Connecting to the robot requires {', '.join(missing)} to be specified.")


def create_robot(
    args: argparse.ArgumentParser, charuco: cv2.aruco_CharucoBoard, aruco_dict: cv2.aruco_Dictionary
) -> Tuple[AutomaticCameraCalibrationRobot, argparse.Namespace]:
    _require_robot_credentials(args)
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


def calibrate_spot_hand() -> None:
    parser = create_robot_parser()
    args, aruco_dict, charuco = setup_calibration_param(parser)

    # Collect new data and calibrate
    if not args.from_data and not args.from_yaml:
        in_hand_bot, args = create_robot(args, charuco=charuco, aruco_dict=aruco_dict)

        logger.warning("This script moves the robot around. !!! USE AT YOUR OWN RISK !!!")
        logger.warning("HOLD Ctrl + C NOW TO CANCEL")
        logger.warning("The calibration board should be about a meter away with nothing within a meter of the robot.")
        logger.warning("The robot should NOT be docked, and nobody should have robot control")
        logger.warning(f"the ip is: {args.ip}")
        input("Press Enter to continue...")

        images, poses = get_multiple_perspective_camera_calibration_dataset(
            auto_cam_cal_robot=in_hand_bot,
            max_num_images=args.max_num_images,
            distances_z=np.arange(*args.dist_from_board_viewpoint_range),
            distances_x=np.arange(*args.dist_along_board_width),
            x_axis_rots=np.arange(*args.x_axis_rot_viewpoint_range),
            y_axis_rots=np.arange(*args.y_axis_rot_viewpoint_range),
            z_axis_rots=np.arange(*args.z_axis_rot_viewpoint_range),
            use_degrees=args.use_degrees,
            settle_time=args.settle_time,
            data_path=args.data_path,
            save_data=args.save_data,
        )

        calibration = calibration_helper(
            images=images, args=args, charuco=charuco, aruco_dict=aruco_dict, poses=poses, result_path=args.result_path
        )
        if args.save_to_robot:
            logger.info("Saving calibration to robot...")
            in_hand_bot.write_calibration_to_robot(calibration)
        in_hand_bot.shutdown()

    # Load and send previously computed and saved calibration data to the robot
    # Assumes the user wants to send the calibration to the robot so the -send flag is not needed/checked
    elif args.from_yaml:
        try:
            in_hand_bot, args = create_robot(args, charuco=charuco, aruco_dict=aruco_dict)
            with open(args.result_path, "r") as file:
                calibration = yaml.safe_load(file)
                send_to_robot = input(
                    f"Loaded calibration data:\n{calibration}\nDo you want to send this calibration to the robot?"
                    " (y/n): "
                )
                if send_to_robot.strip().lower() == "y":
                    logger.info("Saving calibration to robot...")
                    in_hand_bot.write_calibration_to_robot(calibration)
                else:
                    logger.info("Calibration not sent to robot. Shutting down.")
        except Exception as e:
            raise ValueError(f"Failed to load calibration from {args.data_path}:\n{e}\n")
    # Load previously collected data and compute calibration
    else:
        logger.info(f"Loading images from {args.data_path}")
        images, poses = load_dataset_from_path(args.data_path)
        calibration = calibration_helper(
            images=images, args=args, charuco=charuco, aruco_dict=aruco_dict, poses=poses, result_path=args.result_path
        )
        if args.save_to_robot:
            logger.info("Connecting to robot...")
            in_hand_bot, args = create_robot(args, charuco=charuco, aruco_dict=aruco_dict)
            logger.info("Saving calibration to robot...")
            in_hand_bot.write_calibration_to_robot(calibration)

    logger.info("Calibration complete!")


if __name__ == "__main__":
    calibrate_spot_hand()
