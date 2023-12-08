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
        self.robot_state = RobotState()
        self.robot_metrics = RobotMetrics()
        self.hardware_configuration = HardwareConfiguration()

    def GetRobotState(
        self, request: RobotStateRequest, context: grpc.ServicerContext
    ) -> RobotStateResponse:
        response = RobotStateResponse()
        response.robot_state.CopyFrom(self.robot_state)
        return response

    def GetRobotMetrics(
        self, request: RobotMetricsRequest, context: grpc.ServicerContext
    ) -> RobotMetricsResponse:
        response = RobotMetricsResponse()
        response.robot_metrics.CopyFrom(self.robot_metrics)
        return response

    def GetRobotHardwareConfiguration(
        self, request: RobotHardwareConfigurationRequest, context: grpc.ServicerContext
    ) -> RobotHardwareConfigurationResponse:
        response = RobotHardwareConfigurationResponse()
        response.hardware_configuration.CopyFrom(self.hardware_configuration)
        return response

    def GetRobotLinkModel(
        self, request: RobotLinkModelRequest, context: grpc.ServicerContext
    ) -> RobotLinkModelResponse:
        response = RobotLinkModelResponse()
        response.link_model.filename = ""
        return response
