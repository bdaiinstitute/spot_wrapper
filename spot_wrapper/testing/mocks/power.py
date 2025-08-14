# Copyright (c) 2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

from typing import Any

import grpc
from bosdyn.api.power_pb2 import (
    FanPowerCommandFeedbackRequest,
    FanPowerCommandFeedbackResponse,
    FanPowerCommandRequest,
    FanPowerCommandResponse,
    PowerCommandFeedbackRequest,
    PowerCommandFeedbackResponse,
    PowerCommandRequest,
    PowerCommandResponse,
    PowerCommandStatus,
)
from bosdyn.api.power_service_pb2_grpc import PowerServiceServicer
from bosdyn.api.robot_state_pb2 import PowerState

from spot_wrapper.testing.mocks.robot_state import MockRobotStateService


class MockPowerService(PowerServiceServicer, MockRobotStateService):
    """A mock Spot power service."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.robot_state.power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF

    def PowerCommand(self, request: PowerCommandRequest, context: grpc.ServicerContext) -> PowerCommandResponse:
        response = PowerCommandResponse()
        if request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS:
            self.robot_state.power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
            response.status = PowerCommandStatus.STATUS_SUCCESS
            response.power_command_id = 1
        elif request.request == PowerCommandRequest.Request.REQUEST_OFF_MOTORS:
            self.robot_state.power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF
            response.status = PowerCommandStatus.STATUS_SUCCESS
            response.power_command_id = 1
        else:
            response.status = PowerCommandStatus.STATUS_INTERNAL_ERROR
        return response

    def PowerCommandFeedback(
        self, request: PowerCommandFeedbackRequest, context: grpc.ServicerContext
    ) -> PowerCommandFeedbackResponse:
        response = PowerCommandFeedbackResponse()
        response.status = PowerCommandStatus.STATUS_SUCCESS
        response.power_command_id = request.power_command_id
        return response

    def FanPowerCommand(
        self, request: FanPowerCommandRequest, context: grpc.ServicerContext
    ) -> FanPowerCommandResponse:
        response = FanPowerCommandResponse()
        response.status = FanPowerCommandResponse.Status.STATUS_OK
        response.command_id = 2
        return response

    def FanPowerCommandFeedback(
        self, request: FanPowerCommandFeedbackRequest, context: grpc.ServicerContext
    ) -> FanPowerCommandFeedbackResponse:
        response = FanPowerCommandFeedbackResponse()
        response.status = FanPowerCommandFeedbackResponse.Status.STATUS_COMPLETE
        return response
