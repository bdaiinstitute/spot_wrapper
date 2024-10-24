# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

import logging
from time import sleep
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np
from bosdyn.api import estop_pb2, gripper_camera_param_pb2
from bosdyn.client import create_standard_sdk, math_helpers
from bosdyn.client.estop import (
    EstopClient,
    EstopEndpoint,
    EstopKeepAlive,
    MotorsOnError,
)
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    HAND_FRAME_NAME,
    get_a_tform_b,
)
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import (
    LeaseClient,
    LeaseKeepAlive,
    ResourceAlreadyClaimedError,
)
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    block_until_arm_arrives,
    blocking_sit,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimedOutError

from spot_wrapper.calibration.automatic_camera_calibration_robot import (
    AutomaticCameraCalibrationRobot,
)
from spot_wrapper.calibration.calibration_util import (
    convert_camera_t_viewpoint_to_origin_t_planning_frame,
    est_camera_t_charuco_board_center,
)

logging.basicConfig(
    level=logging.INFO,
)

logger = logging.getLogger(__name__)


class SpotInHandCalibration(AutomaticCameraCalibrationRobot):
    def __init__(self, ip: str, username: str, password: str):
        """
        Calibrated intrinsic used to localize the board once in
        localize_target_to_start_pose_vision . If the board is placed at a known location
        relative to the arm, then there is no need to do this visually, and thus
        no need to estimate intrinsic parameters prior to calibration.
        """
        logger.info("Connecting to robot...")
        sdk = create_standard_sdk("SpotSDK")
        if ip is None or username is None or password is None:
            raise AttributeError("Must specify IP, username, and password for calibration.")
        self.robot = sdk.create_robot(ip)
        self.robot.authenticate(username, password)
        try:
            self.robot.time_sync.wait_for_sync()
        except TimedOutError as e:
            raise ValueError(f"Could not establish a time sync to the robot... {e}")
        logger.info("Established time sync, resetting e-stop...")
        estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        estop_endpoint = EstopEndpoint(client=estop_client, estop_timeout=20, name="my_estop")
        try:
            estop_endpoint.force_simple_setup()
        except MotorsOnError:
            raise ValueError(
                f"You need to disconnect from the robot, cannot setup estop while motors on {MotorsOnError}"
            )
        estop_keepalive = EstopKeepAlive(estop_endpoint)
        estop_keepalive.allow()
        logger.info("E-Stop is reset and robot is allowed to operate")
        client = self.robot.ensure_client(EstopClient.default_service_name)
        if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
            error_message = "Robot is estopped. Use an external E-Stop client, such as the controller to undo"
            self.robot.logger.error(error_message)
            raise Exception(error_message)
        logger.info("Succesfuly reset estop.")
        if not self.robot.has_arm():
            raise ValueError("Could not find an arm to use for the hand calibration.")
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.robot_state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        try:
            self.keep_alive_leave = LeaseKeepAlive(
                self.lease_client,
                must_acquire=True,
                return_at_exit=True,
            )
        except ResourceAlreadyClaimedError as e:
            raise ValueError(f"Must relinquish control from robot before running calib: {e}")

        self.image_requests = [
            build_image_request("hand_color_image", quality_percent=100),
            build_image_request("hand_image", quality_percent=100),
        ]
        # Create a GripperCameraParamsClient
        self.gripper_camera_client = self.robot.ensure_client(GripperCameraParamClient.default_service_name)
        # Katie and Gary cooked up right here !!!**** GripperCameraCalibration
        self.write_calibration_to_robot()
        raise ValueError("Did not mean to go this far")


    def write_calibration_to_robot(self, cal=None):
        from bosdyn.api import gripper_camera_param_pb2
        from bosdyn.api.geometry_pb2 import SE3Pose
        from bosdyn.api.image_pb2 import PinholeModel
        ## The following works and is tested to return blank parameters on unset robot
        ############################################################################
        # get_req = gripper_camera_param_pb2.GripperCameraGetParamRequest()
        # cal = self.gripper_camera_client.get_camera_calib(get_req)
        # print(f"Pre-Set Cal (get cam param req): \n {cal}")
        # get_req = gripper_camera_param_pb2.GetGripperCameraCalibrationRequest()
        # cal = self.gripper_camera_client.get_camera_calib(get_req)
        # print(f"Pre-Set Cal (get calib param req): \n{cal}")
        #############################################################################

        def convert_pinhole_intrinsic_to_proto(intrinsic_matrix):
            """Converts a 3x3 intrinsic matrix to a PinholeModel protobuf."""
            pinhole_model = PinholeModel()
            pinhole_model.focal_length_x = intrinsic_matrix[0, 0]
            pinhole_model.focal_length_y = intrinsic_matrix[1, 1]
            pinhole_model.principal_point_x = intrinsic_matrix[0, 2]
            pinhole_model.principal_point_y = intrinsic_matrix[1, 2]
            return pinhole_model

        if cal is None:
            # Ripped from calib file
            cal = {}
            cal["depth_intrinsic"] = np.array([215.78570285824208, 0.0, 113.50899498114939, 0.0, 215.81041358956026,
                                                91.52930962720102, 0.0, 0.0, 1.0]).reshape((3, 3))
            cal["rgb_intrinsic"] = np.array([[541.9417121434435, 0.0, 319.8759813692453, 0.0, 541.9183052872654,
                                                235.87456783983524, 0.0, 0.0, 1.0]]).reshape((3, 3))
            cal["depth_to_rgb"] = np.array([
                [0.0008540850814552694, -0.9999762875319601, 0.006833367579213797, 0.02068056344450553],
                [0.999950571032565, 0.0009217136573264173, 0.009899794724194794, 0.014867920369954562],
                [-0.009905858383852087, 0.006824574545926849, 0.9999276469584919, -0.011403325992053714],
                [0, 0, 0, 1]
            ])
            cal["depth_to_planning_frame"] = np.array([
                [0.13925927199025206, 0.03472611310021725, -0.9896468825968665, 0.012525123175595418],
                [-0.05828697579746701, 0.9979396646727721, 0.02681518460091079, 0.01978439523093831],
                [0.9885390652964254, 0.05394926080815518, 0.14099643130633896, 0.037143569257920686],
                [0, 0, 0, 1]
            ])
            cal["rgb_to_planning_frame"] = np.linalg.inv(cal["depth_to_rgb"]) @ cal["depth_to_planning_frame"]

        # Converting calibration data to protobuf format
        depth_intrinsics_proto = convert_pinhole_intrinsic_to_proto(cal["depth_intrinsic"])
        rgb_intrinsics_proto = convert_pinhole_intrinsic_to_proto(cal["rgb_intrinsic"])
        depth_to_planning_frame_proto = SE3Pose.from_matrix(cal["depth_to_planning_frame"]).to_proto()
        rgb_to_planning_frame_proto = SE3Pose.from_matrix(cal["rgb_to_planning_frame"]).to_proto()

        set_req = gripper_camera_param_pb2.SetGripperCameraCalibrationRequest(
            gripper_cam_cal=gripper_camera_param_pb2.GripperCameraCalibrationProto(
                depth=gripper_camera_param_pb2.GripperDepthCameraCalibrationParams(
                    wr1_tform_sensor=depth_to_planning_frame_proto,
                    intrinsics=gripper_camera_param_pb2.GripperDepthCameraCalibrationParams.DepthCameraIntrinsics(
                        camera_models=gripper_camera_param_pb2.GripperDepthCameraCalibrationParams.DepthCameraIntrinsics.PinholeModel(
                            pinhole=depth_intrinsics_proto
                        )
                    )
                ),
                rgb=gripper_camera_param_pb2.GripperColorCameraCalibrationParams(
                    wr1_tform_sensor=rgb_to_planning_frame_proto,
                    intrinsics=[
                        gripper_camera_param_pb2.GripperColorCameraCalibrationParams.ColorCameraIntrinsics(
                            pinhole=rgb_intrinsics_proto
                        )
                    ]
                )
            )
        )

        # Send the request to the robot
        result = self.gripper_camera_client.set_camera_calib(set_req)
        print(result)


    def capture_images(
        self,
        encodings: Optional[List[int]] = None,
    ) -> Union[List, np.ndarray]:
        if encodings is None:
            encodings = [cv2.IMREAD_COLOR, cv2.IMREAD_GRAYSCALE]
        images = []
        image_responses = self.image_client.get_image(self.image_requests)

        if image_responses:
            if len(encodings) != len(image_responses):
                raise ValueError("Need to specify an encoding for each image request")
            for idx, (response, encoding) in enumerate(zip(image_responses, encodings)):
                image_data = response.shot.image.data
                image_data = np.frombuffer(image_data, np.uint8)
                image_data = cv2.imdecode(image_data, encoding)
                images.append(image_data)
        else:
            raise ValueError(f"Could not obtain desired images {self.image_requests}")

        return np.array(images, dtype=object)

    def localize_target_to_principal_camera(self, images: Union[List, np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        try:
            return est_camera_t_charuco_board_center(
                images[0],
                self.charuco_board,
                self.aruco_dict,
                self.estimated_camera_matrix,
                self.estimated_camera_distort_coeffs,
            )
        except AttributeError as e:
            raise ValueError(f"Must call _set_localization_param prior to localizing: {e}")

    def move_cameras_to_see_calibration_target(self) -> np.ndarray:
        def adjust_standing_height(height: float) -> None:
            stand_command = RobotCommandBuilder.synchro_stand_command(body_height=height)
            self.command_client.robot_command(stand_command)
            logger.info(f"Robot standing height adjusted by {height} meters.")

        logger.info("Powering on robot")
        self.robot.power_on(timeout_sec=20)
        logger.info("About to open claw, stand, and ready arm to collect calibration set")
        claw_open_command = RobotCommandBuilder.claw_gripper_open_command()
        self.command_client.robot_command(claw_open_command)
        blocking_stand(self.command_client)
        adjust_standing_height(height=-0.2)
        ready_arm_command = RobotCommandBuilder.arm_ready_command()
        ready_id = self.command_client.robot_command(ready_arm_command)
        block_until_arm_arrives(self.command_client, ready_id)
        logger.info("Robot in ready position for calibration dataset")
        logger.info("About to lower arm to see board better...")
        transform_offset = np.eye(4)
        transform_offset[:-1, -1] = np.array([0.0, 0.3, -0.2]).reshape((3,))
        old_pose, ready_pose = self.offset_cameras_from_current_view(transform_offset=transform_offset)
        return ready_pose

    def offset_cameras_from_current_view(
        self,
        transform_offset: np.ndarray,
        origin_t_planning_frame: Optional[np.ndarray] = None,
        use_body: bool = False,
        duration_sec: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        def grab_state_as_transform() -> np.ndarray:
            robot_state = self.robot_state_client.get_robot_state()
            origin_t_planning_frame = get_a_tform_b(
                robot_state.kinematic_state.transforms_snapshot,
                BODY_FRAME_NAME,
                HAND_FRAME_NAME,
            )
            pose_transform = np.eye(4)
            pose_transform[:-1, -1] = np.array(
                [
                    origin_t_planning_frame.x,
                    origin_t_planning_frame.y,
                    origin_t_planning_frame.z,
                ]
            ).reshape((3,))
            pose_transform[:-1, :-1] = origin_t_planning_frame.rot.to_matrix()
            return pose_transform

        if origin_t_planning_frame is None:
            initial_pose = grab_state_as_transform()
        else:
            initial_pose = origin_t_planning_frame

        try:
            opencv_camera_t_viewpoint = np.eye(4)
            opencv_camera_t_viewpoint[:3, :3] = self.hand_t_opencv_camera_rot
        except AttributeError:
            raise ValueError("Must call _set_localization_param prior to invoking this method.")
        new_pose = convert_camera_t_viewpoint_to_origin_t_planning_frame(
            origin_t_planning_frame=initial_pose,
            planning_frame_t_opencv_camera=opencv_camera_t_viewpoint,
            opencv_camera_t_viewpoint=transform_offset,
        )

        new_pose_quat = math_helpers.Quat.from_matrix(new_pose[:-1, :-1])

        new_arm_command = RobotCommandBuilder.arm_pose_command(
            new_pose[0, 3],  # x
            new_pose[1, 3],  # y
            new_pose[2, 3],  # z
            new_pose_quat.w,
            new_pose_quat.x,
            new_pose_quat.y,
            new_pose_quat.z,
            BODY_FRAME_NAME,
            duration_sec,
        )
        if not use_body:
            command_id = self.command_client.robot_command(new_arm_command)
        else:
            follow_arm_command = RobotCommandBuilder.follow_arm_command()
            command = RobotCommandBuilder.build_synchro_command(follow_arm_command, new_arm_command)
            command_id = self.command_client.robot_command(command)
        block_until_arm_arrives(
            self.command_client,
            command_id,
            timeout_sec=duration_sec * 2,
        )
        sleep(duration_sec * 0.5)  # settle before grabbing new state for better quality
        return (initial_pose, grab_state_as_transform())  # second value is new value

    def shutdown(self) -> None:
        stow_arm_command = RobotCommandBuilder.arm_stow_command()
        ready_id = self.command_client.robot_command(stow_arm_command)
        block_until_arm_arrives(self.command_client, ready_id)
        blocking_sit(self.command_client)
        self.keep_alive_leave.shutdown()
        logger.info("IT IS NOW SAFE TO RETAKE CONTROL OF SPOT, DONE TAKING PICS :)")

    def _set_localization_param(
        self,
        charuco_board: cv2.aruco_CharucoBoard,
        aruco_dict: cv2.aruco_Dictionary,
        resolution: Tuple[int, int] = (
            640,
            480,
        ),  # (1920, 1080),#(640, 480),
        hand_t_opencv_camera_rot: np.ndarray = np.array(
            [
                [0.00, 0.00, 1.00],
                [-1.00, 0.00, 0.00],
                [0.00, -1.00, 0.00],
            ]
        ),
    ) -> None:
        self.resolution = resolution

        gripper_camera_params = gripper_camera_param_pb2.GripperCameraParamRequest()
        if resolution == (640, 480):
            camera_matrix_est = np.array(  # hard coded for initial localization
                [
                    [540.70883089, 0.0, 322.53051396],
                    [0.0, 540.45766656, 236.25863472],
                    [0.0, 0.0, 1.0],
                ]
            )
            gripper_camera_params.params.camera_mode = 11
        elif resolution == (1920, 1080):
            camera_matrix_est: np.ndarray = np.array(
                [
                    [1656.0873036483201, 0.0, 960.0],
                    [0.0, 1656.0873036483201, 540.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            gripper_camera_params.params.camera_mode = 14
        else:
            raise ValueError(f"Unsupported resolution for board localization. {resolution}")
        logger.info(f"Setting gripper camera resolution to be {resolution}")
        self.gripper_camera_client.set_camera_params(gripper_camera_params)
        logger.info(f"Resolution set succesfully to {resolution}")
        camera_dist_coeffs_est: np.ndarray = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])

        self.charuco_board = charuco_board
        self.aruco_dict = aruco_dict
        self.estimated_camera_matrix = camera_matrix_est
        self.estimated_camera_distort_coeffs = camera_dist_coeffs_est
        # this can be obtained with your favorite transform lib like SO3.Eul([0, 90, -90]
        self.hand_t_opencv_camera_rot = hand_t_opencv_camera_rot
