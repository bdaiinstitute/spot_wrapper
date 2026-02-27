# Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

import argparse
import logging
from typing import Tuple

import cv2
import numpy as np
import synchros2.process as ros_process
import yaml

from spot_wrapper.calibration.automatic_camera_calibration_robot import AutomaticCameraCalibrationRobot
from spot_wrapper.calibration.calibration_clis import (
    calibrate_robot_cli,
    ext_cli,
    setup_calibration_param,
    spot_cli,
)
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
    in_hand_bot, args = create_robot(args, charuco=charuco, aruco_dict=aruco_dict)

    # Collect new data and calibrate
    if not args.from_data:
        logger.warning("This script moves the robot around. !!! USE AT YOUR OWN RISK !!!")
        logger.warning("HOLD Ctrl + C NOW TO CANCEL")
        logger.warning("The calibration board should be about a meter away with nothing within a meter of the robot.")
        logger.warning("The robot should NOT be docked, and nobody should have robot control")
        input("Press Enter to continue...")
        # sleep(5)

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
    # Send previously computed and saved calibration data to the robot
    elif args.from_yaml:
        try:
            with open(args.data_path, "r") as file:
                calibration = yaml.safe_load(file)
                logger.info(f"Loaded calibration data:\n{calibration}")
                if args.save_to_robot:
                    logger.info("Saving calibration to robot...")
                    in_hand_bot.write_calibration_to_robot(calibration)
        except Exception as e:
            raise ValueError(f"Failed to load calibration from {args.data_path}: {e}\nIs it a calibration yaml file?")
    # Load previously collected data and compute calibration
    else:
        logger.info(f"Loading images from {args.data_path}")
        images, poses = load_dataset_from_path(args.data_path)
        calibration = calibration_helper(
            images=images, args=args, charuco=charuco, aruco_dict=aruco_dict, poses=poses, result_path=args.result_path
        )
        if args.save_to_robot:
            logger.info("Saving calibration to robot...")
            in_hand_bot.write_calibration_to_robot(calibration)

    logger.info("Calibration complete!")


@ros_process.main(ext_cli())
def main(args: argparse.Namespace) -> None:
    spot_main()
