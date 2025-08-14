# Copyreference (c) 2024 Boston Dynamics AI Institute LLC. All references reserved.

# What's below can be used in standalone tool
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple, Union

import numpy as np


class AutomaticCameraCalibrationRobot(ABC):
    @abstractmethod
    def capture_images(self) -> Union[List, np.ndarray]:
        """
        Capture images from the cameras you wish to calibrate at the same moment in time
        (time-synchronized). Then, construct either a list or a NumPy array
        of these images. This will be called every time that your robot reaches a new
        calibration viewpoint, so ensure that image order to camera correlation is consistent
        across all calls. Also, it is important to ensure that the photos are as time-synchronized
        as possible, with as little motion blur as possible.

        Returns:
            Union[List, np.ndarray]: The time-synchronized images from all relevant cameras.
        """
        pass

    @abstractmethod
    def localize_target_to_principal_camera(self, images: Union[List, np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Localize the charuco board relative to what is considered
        the principal camera for the calibration. This could be done
        either visually, or via kinematics and fixing the board relative
        to the robot. Visually is more flexible, although this
        implies some knowledge of the principal cameras intrinsics.

        In this case, principal camera could be an actual camera, or could also be a "virtual"
        camera. The principal camera's frame is just what frame is used to sample viewpoints in.

        The board pose's translation should be at the center of the board, with the orientation
        in OpenCV format, where the +Z points out of the board with
        the other axis being parallel to the sides of the board.

        Your camera pose should be in OpenCV/ROS convention, where
        # +x should point to the right in the image
        # +y should point down in the image
        # +z should point into to plane of the image

        If you'd like to do this through visual localization, you can use

        def est_camera_t_charuco_board_center(
                img: np.ndarray,
                charuco_board: cv2.aruco_CharucoBoard,
                aruco_dict: cv2.aruco_Dictionary,
                camera_matrix: np.ndarray,
                dist_coeffs: np.ndarray,
            )
        from calibration_util.py

        Args:
            images (Union[List, np.ndarray]): the images that could be used to determine
                how far the target is from the "principal" camera

        Returns:
            Tuple[np.ndarray, np.ndarray]: _description_
        """
        pass

    @abstractmethod
    def move_cameras_to_see_calibration_target(self) -> np.ndarray:
        """
        This is a robot specific sequence to start the calibration. This
        could be as simple as a heuristic to move the robot to a known pose where
        it is generally looking at the checkerboard, or this could be a more sophisticated
        sequence to look around in the scene until a board is found
        (for example, grabbing images with self.capture_images(),
        and checking if a board is found with detect_charuco_corners from calibration_util.py)

        Returns:
            np.ndarray: the 4x4 homogenous transform of the starting pose where the calibration
                board is visible.
        """
        pass

    @abstractmethod
    def offset_cameras_from_current_view(
        self,
        transform_offset: np.ndarray,
        origin_t_planning_frame: Optional[np.ndarray] = None,
        duration_sec: float = 1.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Move the robot to a desired position, such that the cameras move by transform_offset.
        This is how the robot visits desired viewpoints.

        IMPORTANT: You likely have to convert transform_offset from the "principal"
        camera frame into the planning frame (if the camera frame isn't the planning frame).
        This can be done by calling

        convert_camera_t_viewpoint_to_origin_t_planning_frame(
            origin_t_planning_frame=origin_t_planning_frame,
            planning_frame_t_opencv_camera=HOMOGENOUS_TRANSFORM_FROM_PLANNING_FRAME_TO_OPENCV_CAM,
            opencv_camera_t_viewpoint=transform_offset,
        )
        from calibration_util.py

        Your camera pose should be in OpenCV/ROS convention, where
        +x should point to the right in the image
        +y should point down in the image
        +z should point into to plane of the image

        Args:
            transform_offset (np.ndarray): the 4x4 homogenous transform of
                how far you'd like to shift your cameras
            origin_t_planning_frame (Optional[np.ndarray], optional): the 4x4 homogenous
                transform of your robot origin to planning frame. Could be hard-coded
                or read off of the robot. Defaults to None.
            duration_sec (float, optional): How many seconds provided to execute
                the move. Defaults to 1.0.

        Returns:
            Tuple[np.ndarray, np.ndarray]: the 4x4 homogenous transform of origin_t_planning_frame
                before the move, and the 4x4 homogenous transform of origin_t_planning_frame
                after the move.
        """
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """
        Cleanup the robot connection, and disconnect so that other programs can resume
        control of the robot while this program continues to solve the calibration parameters.
        """
        pass
