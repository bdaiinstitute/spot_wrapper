# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

import argparse
import ast
import logging
from typing import List, Tuple, Union

import cv2
import numpy as np

from spot_wrapper.calibration.calibration_util import (
    load_images_from_path,
    multistereo_calibration_charuco,
    save_calibration_parameters,
)

logging.basicConfig(
    level=logging.INFO,
)
logger = logging.getLogger(__name__)


def calibration_helper(
    images: Union[List[np.ndarray], np.ndarray],
    args: argparse.Namespace,
    charuco: cv2.aruco_CharucoBoard,
    aruco_dict: cv2.aruco_Dictionary,
):
    logger.warning(
        f"Calibrating from {len(images)} images.. for every "
        f"{args.photo_utilization_ratio} recorded photos 1 is used to calibrate"
    )
    calibration = multistereo_calibration_charuco(
        images[:: args.photo_utilization_ratio],
        desired_stereo_pairs=args.stereo_pairs,
        charuco_board=charuco,
        aruco_dict=aruco_dict,
    )
    logger.info(f"Finished script, obtained {calibration}")
    logger.info("Saving calibration param")
    save_calibration_parameters(
        data=calibration,
        output_path=args.result_path,
        num_images=len(images[:: args.photo_utilization_ratio]),
        tag=args.tag,
        parser_args=args,
        unsafe=args.unsafe_tag_save,
    )


def main():
    parser = calibrator_cli()
    args = parser.parse_args()
    if hasattr(cv2.aruco, args.dict_size):
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict_size))
    else:
        raise ValueError(f"Invalid ArUco dictionary: {args.dict_size}")
    charuco = cv2.aruco.CharucoBoard_create(
        args.num_checkers_width,
        args.num_checkers_height,
        args.checker_dim,
        args.marker_dim,
        aruco_dict,
    )
    logger.info(f"Loading images from {args.data_path}")
    images = load_images_from_path(args.data_path)
    calibration_helper(images=images, args=args, charuco=charuco, aruco_dict=aruco_dict)


def calibrator_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser("Calibrate Eye-In Hand Camera System :)")
    # Calculations
    parser.add_argument(
        "--stereo_pairs",
        "-sp",
        type=parse_tuple_list,
        required=True,
        default=[(1, 0)],
        help=(
            "Capture images returns a list of images. Stereo pairs correspond to"
            "what index in the list of images' corresponding camera"
            "to calibrate to what camera. Say capture_images returns [rgb_img, depth_img]"
            "and you want to register depth to rgb, then the desired stereo pair"
            'is "[(1,0)]". If you want to register more than one pair, you can do it like "[(1, 0), (2,0)]."'
            "Make sure to put the stereo pairs in quotes so bash doesn't complain"
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
        help="How many checkers wide is your board",
        default=9,
    )

    parser.add_argument(
        "--num_checkers_height",
        "-nch",
        dest="num_checkers_height",
        type=int,
        help="How many checkers tall is your board",
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
        "--checker_dim",
        "-cd",
        dest="checker_dim",
        type=float,
        default=0.115,
        help="Checker size in meters",
    )

    parser.add_argument(
        "--marker_dim",
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
        help="The path in which to save images",
        default=None,
    )

    parser.add_argument(
        "--result_path",
        "-rp",
        dest="result_path",
        type=str,
        required=True,
        help="Where to store calibration result as file",
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

    return parser


# collection
def parse_tuple_list(string: str) -> List[Tuple[int, int]]:
    try:
        # Use ast.literal_eval to safely evaluate the string as a Python expression
        value = ast.literal_eval(string)
        if isinstance(value, list) and all(isinstance(t, tuple) and len(t) == 2 for t in value):
            return value
        else:
            raise argparse.ArgumentTypeError(f"Expected a list of tuples, but got: {string}")
    except (ValueError, SyntaxError):
        raise argparse.ArgumentTypeError(f"Invalid tuple list format: {string}")


if __name__ == "__main__":
    main()
