# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import logging

import grpc
import pytest
from bosdyn.api.manipulation_api_pb2 import (
    ManipulationApiRequest,
    ManipulationApiResponse,
)
from bosdyn.api.power_pb2 import (
    PowerCommandRequest,
    PowerCommandResponse,
    PowerCommandStatus,
)
from bosdyn.api.robot_command_pb2 import RobotCommandResponse
from bosdyn.api.robot_state_pb2 import PowerState

import spot_wrapper.testing
from spot_wrapper.testing.fixtures import SpotFixture
from spot_wrapper.testing.mocks import MockSpot
from spot_wrapper.wrapper import SpotWrapper


@spot_wrapper.testing.fixture(ids=["with_arm", "without_arm"], params=[True, False])
class simple_spot(MockSpot):
    # Inheriting from spot_wrapper.testing.mocks.MockSpot
    # takes care of providing enough dummy base services
    # for SpotWrapper instantiation to be possible.

    def __init__(self, request) -> None:
        # Fixture initialization can request other fixtures.
        super().__init__()
        if request.param:
            manipulator_state = self.robot_state.manipulator_state
            manipulator_state.is_gripper_holding_item = True

    def PowerCommand(
        self, request: PowerCommandRequest, context: grpc.ServicerContext
    ) -> PowerCommandResponse:
        # Provide custom bosdyn.api.PowerService/PowerCommand implementation.
        response = PowerCommandResponse()
        power_state = self.robot_state.power_state
        if request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS:
            power_state.motor_power_state = (
                PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
            )
            response.status = PowerCommandStatus.STATUS_SUCCESS
        elif request.request == PowerCommandRequest.Request.REQUEST_OFF_MOTORS:
            power_state.motor_power_state = (
                PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF
            )
            response.status = PowerCommandStatus.STATUS_SUCCESS
        else:
            response.status = PowerCommandStatus.STATUS_INTERNAL_ERROR
        return response


@pytest.fixture
def simple_spot_wrapper(simple_spot: SpotFixture) -> SpotWrapper:
    # Use fixture address and port for wrapper.
    return SpotWrapper(
        username="spot",
        password="spot",
        hostname=simple_spot.address,
        port=simple_spot.port,
        robot_name=simple_spot.api.name,
        logger=logging.getLogger("spot"),
    )


def test_wrapper_setup(
    simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper
) -> None:
    # spot_wrapper.testing.mocks.MockSpot dummy services enable basic usage.
    assert simple_spot_wrapper.is_valid

    ok, message = simple_spot_wrapper.claim()
    assert ok, message

    assert simple_spot_wrapper.id is not None
    assert simple_spot_wrapper.id.nickname == simple_spot.api.name

    if simple_spot_wrapper.has_arm():
        # bosdyn.api.ManipulationApiService/ManipulationApi implementation
        # is mocked (as spot_wrapper.testing.mocks.MockSpot is automatically
        # specified), so we provide a response in advance.
        request = ManipulationApiRequest()
        request.pick_object.frame_name = "gripper"
        request.pick_object.object_rt_frame.x = 1.0

        response = ManipulationApiResponse()
        response.manipulation_cmd_id = 1
        simple_spot.api.ManipulationApi.future.returns(response)

        ok, message, command_id = simple_spot_wrapper.manipulation_command(request)
        assert ok and response.manipulation_cmd_id == command_id, message

    # Power toggling relies on a combination of
    assert not simple_spot_wrapper.check_is_powered_on()
    assert simple_spot_wrapper.toggle_power(True)

    # bosdyn.api.RobotCommandService/RobotCommand implementation
    # is mocked (as spot_wrapper.testing.mocks.MockSpot is
    # automatically specified), so we provide a response in
    # advance.
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response)
    ok, message = simple_spot_wrapper.sit()
    assert ok, message
