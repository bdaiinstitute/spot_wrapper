#!/usr/bin/env python3

# Copyright (c) 2025-2026 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.


import argparse
import logging
from typing import Tuple

import cv2

from spot_wrapper.calibration.charuco_board_detection import (
    charuco_pose_sanity_check,
    create_charuco_board,
    create_ideal_charuco_image,
    detect_charuco_corners,
)

logging.basicConfig(
    level=logging.INFO,
)

logger = logging.getLogger(__name__)


def setup_calibration_param(
    parser: argparse.ArgumentParser,
) -> Tuple[argparse.Namespace, cv2.aruco_Dictionary, cv2.aruco_CharucoBoard]:
    """Set up calibration parameters from command line arguments.

    Args:
        parser (argparse.ArgumentParser): The argument parser to set up from command line.

    Raises:
        ValueError: If the provided ArUco dictionary is invalid.

    Returns:
        Tuple[argparse.Namespace, cv2.aruco_Dictionary, cv2.aruco_CharucoBoard]:
        The parsed arguments, ArUco dictionary, and Charuco board.
    """
    args = parser.parse_args()
    if hasattr(cv2.aruco, args.dict_size):
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict_size))
    else:
        raise ValueError(f"Invalid ArUco dictionary: {args.dict_size}")
    charuco = create_charuco_board(
        num_checkers_width=args.num_checkers_width,
        num_checkers_height=args.num_checkers_height,
        checker_dim=args.checker_dim,
        marker_dim=args.marker_dim,
        aruco_dict=aruco_dict,
        legacy=args.legacy_charuco_pattern,
    )

    if not args.allow_default_internal_corner_ordering:
        logger.warning("Enforcing bottom up charuco ordering. Pre-computing correlation now...")
        detect_charuco_corners(
            create_ideal_charuco_image(charuco_board=charuco),
            charuco_board=charuco,
            aruco_dict=aruco_dict,
            enforce_ascending_ids_from_bottom_left_corner=True,
        )
    if args.show_board_pattern:
        logger.warning("Checking board, you'll need to close a window in a sec (press any key)")
        charuco_pose_sanity_check(
            create_ideal_charuco_image(charuco_board=charuco, colorful=True),
            charuco_board=charuco,
            aruco_dict=aruco_dict,
        )
    return args, aruco_dict, charuco


def calibrator_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser("CalibrateEyeInHandCameraSystem")
    # Calculations

    parser.add_argument(
        "--legacy_charuco_pattern",
        action="store_true",
        help=(
            "Whether to use the legacy charuco pattern. For Spot Default board, this should be set."
            "If you aren't sure if your board is legacy, try supplying the --show_board_pattern"
            "arg to verify that the cv2 board matches your board."
        ),
    )
    parser.add_argument(
        "--show_board_pattern",
        action="store_true",
        default=False,
        help="Whether to visually verify the board pattern (to check legacy and internal corner ordering.",
    )

    parser.add_argument(
        "--allow_default_internal_corner_ordering",
        action="store_true",
        default=False,
        help=(
            "Whether to allow default internal corner ordering. "
            "For new versions of OpenCV, it is recommended "
            "to NOT set this parameter to make sure that corners "
            "are ordered in a known pattern. "
            "Try supplying the --show_board_pattern flag "
            "to check whether you should enable this flag "
            "When checking, Z-Axis should point out of board. "
        ),
    )

    parser.add_argument(
        "--photo_utilization_ratio",
        "-pur",
        dest="photo_utilization_ratio",
        type=int,
        default=1,
        help=(
            "Photos that are collected/loaded vs. used for calibration are in a 1 to"
            "photo utilization ratio. For getting a rough guess on cheaper hardware"
            "without losing collection fidelity. For example, set to 2 to only use half the photos."
        ),
    )

    # calibration param
    parser.add_argument(
        "--num_checkers_width",
        "-ncw",
        dest="num_checkers_width",
        type=int,
        help="How many checkers is the widest dimension of your board",
        default=9,
    )

    parser.add_argument(
        "--num_checkers_height",
        "-nch",
        dest="num_checkers_height",
        type=int,
        help="How many checkers is the other dimension of your board",
        default=4,
    )

    parser.add_argument(
        "--dict_size",
        type=str,
        choices=[
            "DICT_4X4_50",
            "DICT_4X4_100",
            "DICT_4X4_250",
            "DICT_4X4_1000",
            "DICT_5X5_50",
            "DICT_5X5_100",
            "DICT_5X5_250",
            "DICT_5X5_1000",
            "DICT_6X6_50",
            "DICT_6X6_100",
            "DICT_6X6_250",
            "DICT_6X6_1000",
            "DICT_7X7_50",
            "DICT_7X7_100",
            "DICT_7X7_250",
            "DICT_7X7_1000",
            "DICT_ARUCO_ORIGINAL",
        ],
        default="DICT_4X4_50",
        help="Choose the ArUco dictionary size.",
    )

    parser.add_argument(
        "--checker_dim_meters",
        "-cd",
        dest="checker_dim",
        type=float,
        default=0.115,
        help="Checker size in meters",
    )

    parser.add_argument(
        "--marker_dim_meters",
        "-md",
        dest="marker_dim",
        type=float,
        default=0.09,
        help="Aruco Marker size in meters",
    )

    # path and saving
    parser.add_argument(
        "--data_path",
        "-dp",
        dest="data_path",
        type=str,
        help="The absolute path in which to save images",
        default=None,
    )

    parser.add_argument(
        "--result_path",
        "-rp",
        dest="result_path",
        type=str,
        required=False,
        help="The absolute path where to store calibration result file",
    )

    parser.add_argument(
        "--tag",
        "-t",
        dest="tag",
        type=str,
        required=False,
        default="default",
        help=(
            "What heading to put for the calibration in the config file."
            "If this is your first time running, the tag should be set to default "
            "for the sake of interoperability with other functionality."
            "If this is a shared config file with other people, perhaps put"
            "a unique identifier, or default, if you'd like to override"
            "for everyone."
        ),
    )

    parser.add_argument(
        "--unsafe_tag_save",
        action="store_true",
        help="If set, skips safety checks for tagging calibration.",
    )

    parser.add_argument(
        "--stereo_pairs",
        "-sp",
        dest="stereo_pairs",
        nargs="+",
        type=lambda s: tuple(int(x) for x in s.split(",")),
        default=[(0, 1)],
        help=(
            "Stereo camera pairs to calibrate, as comma-separated index pairs. "
            "E.g. '0,1' for a single stereo pair between camera 0 and camera 1. "
            "Defaults to [(0, 1)]."
        ),
    )

    return parser


def calibrate_robot_cli(parser: argparse.ArgumentParser | None = None) -> argparse.ArgumentParser:
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

    parser.add_argument(
        "--from_data",
        "-fd",
        dest="from_data",
        action="store_true",
        help="Whether to only calibrate from recorded dataset on file.",
    )

    parser.add_argument(
        "--dist_along_board_width",
        "-dabw",
        nargs="+",
        type=float,
        dest="dist_along_board_width",
        default=[-0.2, 0.3, 0.1],
        help=(
            "What distances to conduct calibrations at relative to the board (along the board width / X axis). "
            "Three value array arg defines the [Start, Stop), step for the viewpoint sweep."
        ),
    )

    parser.add_argument(
        "--save_data",
        action="store_true",
        dest="save_data",
        default=False,
        help="Whether to save the collected image dataset to data_path.",
    )

    parser.add_argument(
        "--from_yaml",
        "-yaml",
        dest="from_yaml",
        action="store_true",
        help=(
            "Whether the data is from a yaml file. Use this and the '--from_data' and '--send' args to send a"
            " previously saved calibration yaml to the robot"
        ),
    )

    return parser


def spot_cli(parser: argparse.ArgumentParser | None = None) -> argparse.ArgumentParser:
    if parser is None:
        parser = calibrate_robot_cli(None)

    parser.add_argument(
        "--robot_name",
        "-rn",
        dest="robot_name",
        type=str,
        help="The name of the Robot to calibrate (auto-detected from robot if not supplied)",
        default=None,
        required=False,
    )

    parser.add_argument(
        "--ip",
        "-i",
        "-ip",
        dest="ip",
        type=str,
        help="The IP address of the Robot to calibrate (required when connecting to the robot)",
        default=None,
        required=False,
    )
    parser.add_argument(
        "--user",
        "-u",
        "--username",
        dest="username",
        type=str,
        help="Robot Username (required when connecting to the robot)",
        default=None,
        required=False,
    )
    parser.add_argument(
        "--pass",
        "-pw",
        "--password",
        dest="password",
        type=str,
        help="Robot Password (required when connecting to the robot)",
        default=None,
        required=False,
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

    parser.add_argument(
        "--save_to_robot",
        "-save",
        dest="save_to_robot",
        action="store_true",
        help="Whether to save the calibration to the robot.",
    )

    return parser
