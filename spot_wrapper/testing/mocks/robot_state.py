# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import grpc
from bosdyn.api.robot_state_pb2 import (
    HardwareConfiguration,
    RobotHardwareConfigurationRequest,
    RobotHardwareConfigurationResponse,
    RobotLinkModelRequest,
    RobotLinkModelResponse,
    RobotMetrics,
    RobotMetricsRequest,
    RobotMetricsResponse,
    RobotState,
    RobotStateRequest,
    RobotStateResponse,
)
from bosdyn.api.robot_state_service_pb2_grpc import RobotStateServiceServicer
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    GROUND_PLANE_FRAME_NAME,
    HAND_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
)

HEAD_FRAME_NAME = "head"
BACK_FRAME_NAME = "back"
LEFT_FRAME_NAME = "left"
RIGHT_FRAME_NAME = "right"
FRONT_LEFT_FRAME_NAME = "frontleft"
FRONT_RIGHT_FRAME_NAME = "frontright"
BACK_CAMERA_FRAME_NAME = "back_fisheye"
WRIST_FRAME_NAME = "arm_link_wr1"
FRONT_LEFT_CAMERA_FRAME_NAME = "frontleft_fisheye"
FRONT_RIGHT_CAMERA_FRAME_NAME = "frontright_fisheye"
LEFT_CAMERA_FRAME_NAME = "left_fisheye"
RIGHT_CAMERA_FRAME_NAME = "right_fisheye"
HAND_CAMERA_FRAME_NAME = "hand_color_image_sensor"


class MockRobotStateService(RobotStateServiceServicer):
    """
    A mock Spot robot state service.

    It exposes robot state, metrics, and hardware configuration to be modified by the user.
    """

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__(**kwargs)
        self._robot_state = RobotState()
        self._robot_metrics = RobotMetrics()
        self._hardware_configuration = HardwareConfiguration()

        transforms_snapshot = self._robot_state.kinematic_state.transforms_snapshot
        world_to_odom_edge = transforms_snapshot.child_to_parent_edge_map[ODOM_FRAME_NAME]
        world_to_odom_edge.parent_tform_child.rotation.w = 1.0

        body_to_back_edge = transforms_snapshot.child_to_parent_edge_map[BACK_FRAME_NAME]
        body_to_back_edge.parent_frame_name = BODY_FRAME_NAME
        body_to_back_edge.parent_tform_child.position.x = -0.5
        body_to_back_edge.parent_tform_child.rotation.w = 1.0

        body_to_head_edge = transforms_snapshot.child_to_parent_edge_map[HEAD_FRAME_NAME]
        body_to_head_edge.parent_frame_name = BODY_FRAME_NAME
        body_to_head_edge.parent_tform_child.position.x = 0.5
        body_to_head_edge.parent_tform_child.rotation.w = 1.0

        body_to_left_edge = transforms_snapshot.child_to_parent_edge_map[LEFT_FRAME_NAME]
        body_to_left_edge.parent_frame_name = BODY_FRAME_NAME
        body_to_left_edge.parent_tform_child.position.y = 0.2
        body_to_left_edge.parent_tform_child.rotation.w = 1.0

        body_to_right_edge = transforms_snapshot.child_to_parent_edge_map[RIGHT_FRAME_NAME]
        body_to_right_edge.parent_frame_name = BODY_FRAME_NAME
        body_to_right_edge.parent_tform_child.position.y = -0.2
        body_to_right_edge.parent_tform_child.rotation.w = 1.0

        head_to_front_left_edge = transforms_snapshot.child_to_parent_edge_map[FRONT_LEFT_FRAME_NAME]
        head_to_front_left_edge.parent_frame_name = HEAD_FRAME_NAME
        head_to_front_left_edge.parent_tform_child.position.y = 0.2
        head_to_front_left_edge.parent_tform_child.rotation.w = 1.0

        head_to_front_right_edge = transforms_snapshot.child_to_parent_edge_map[FRONT_RIGHT_FRAME_NAME]
        head_to_front_right_edge.parent_frame_name = HEAD_FRAME_NAME
        head_to_front_right_edge.parent_tform_child.position.y = -0.2
        head_to_front_right_edge.parent_tform_child.rotation.w = 1.0

        head_to_wrist_edge = transforms_snapshot.child_to_parent_edge_map[WRIST_FRAME_NAME]
        head_to_wrist_edge.parent_frame_name = HEAD_FRAME_NAME
        head_to_wrist_edge.parent_tform_child.position.x = 0.2
        head_to_wrist_edge.parent_tform_child.position.z = 0.5
        head_to_wrist_edge.parent_tform_child.rotation.w = 1.0

        for parent_frame_name, child_frame_name in (
            (ODOM_FRAME_NAME, BODY_FRAME_NAME),
            (BODY_FRAME_NAME, GROUND_PLANE_FRAME_NAME),
            (BODY_FRAME_NAME, VISION_FRAME_NAME),
            (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME),
            (BACK_FRAME_NAME, BACK_CAMERA_FRAME_NAME),
            (LEFT_FRAME_NAME, LEFT_CAMERA_FRAME_NAME),
            (RIGHT_FRAME_NAME, RIGHT_CAMERA_FRAME_NAME),
            (FRONT_LEFT_FRAME_NAME, FRONT_LEFT_CAMERA_FRAME_NAME),
            (FRONT_RIGHT_FRAME_NAME, FRONT_RIGHT_CAMERA_FRAME_NAME),
            (WRIST_FRAME_NAME, HAND_FRAME_NAME),
            (HAND_FRAME_NAME, HAND_CAMERA_FRAME_NAME),
        ):
            edge = transforms_snapshot.child_to_parent_edge_map[child_frame_name]
            edge.parent_frame_name = parent_frame_name
            edge.parent_tform_child.rotation.w = 1.0

    @property
    def robot_state(self) -> RobotState:
        return self._robot_state

    @property
    def robot_metrics(self) -> RobotMetrics:
        return self._robot_metrics

    @property
    def hardware_configuration(self) -> HardwareConfiguration:
        return self._hardware_configuration

    def GetRobotState(self, request: RobotStateRequest, context: grpc.ServicerContext) -> RobotStateResponse:
        response = RobotStateResponse()
        response.robot_state.CopyFrom(self._robot_state)
        return response

    def GetRobotMetrics(self, request: RobotMetricsRequest, context: grpc.ServicerContext) -> RobotMetricsResponse:
        response = RobotMetricsResponse()
        response.robot_metrics.CopyFrom(self._robot_metrics)
        return response

    def GetRobotHardwareConfiguration(
        self, request: RobotHardwareConfigurationRequest, context: grpc.ServicerContext
    ) -> RobotHardwareConfigurationResponse:
        response = RobotHardwareConfigurationResponse()
        response.hardware_configuration.CopyFrom(self._hardware_configuration)
        return response

    def GetRobotLinkModel(
        self, request: RobotLinkModelRequest, context: grpc.ServicerContext
    ) -> RobotLinkModelResponse:
        response = RobotLinkModelResponse()
        response.link_model.filename = ""
        return response
