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


class MockRobotStateService(RobotStateServiceServicer):
    """
    A mock Spot robot state service.

    It exposes robot state, metrics, and hardware configuration to be modified by the user.
    """

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__(**kwargs)
        self._robot_state = RobotState()
        transforms_snapshot = self._robot_state.kinematic_state.transforms_snapshot
        world_to_odom_edge = transforms_snapshot.child_to_parent_edge_map["odom"]
        world_to_odom_edge.parent_tform_child.rotation.w = 1.0
        odom_to_body_edge = transforms_snapshot.child_to_parent_edge_map["body"]
        odom_to_body_edge.parent_frame_name = "odom"
        odom_to_body_edge.parent_tform_child.rotation.w = 1.0
        body_to_vision_edge = transforms_snapshot.child_to_parent_edge_map["vision"]
        body_to_vision_edge.parent_frame_name = "body"
        body_to_vision_edge.parent_tform_child.rotation.w = 1.0
        self._robot_metrics = RobotMetrics()
        self._hardware_configuration = HardwareConfiguration()

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
