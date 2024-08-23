# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import logging
import time
from typing import Iterator, List

import grpc
import pytest
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus, SitCommand, StandCommand
from bosdyn.api.manipulation_api_pb2 import (
    ManipulationApiRequest,
    ManipulationApiResponse,
)
from bosdyn.api.power_pb2 import (
    PowerCommandRequest,
    PowerCommandResponse,
    PowerCommandStatus,
)
from bosdyn.api.robot_command_pb2 import RobotCommandFeedbackResponse, RobotCommandResponse
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

    def PowerCommand(self, request: PowerCommandRequest, context: grpc.ServicerContext) -> PowerCommandResponse:
        # Provide custom bosdyn.api.PowerService/PowerCommand implementation.
        response = PowerCommandResponse()
        power_state = self.robot_state.power_state
        if request.request == PowerCommandRequest.Request.REQUEST_ON_MOTORS:
            power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_ON
            response.status = PowerCommandStatus.STATUS_SUCCESS
        elif request.request == PowerCommandRequest.Request.REQUEST_OFF_MOTORS:
            power_state.motor_power_state = PowerState.MotorPowerState.MOTOR_POWER_STATE_OFF
            response.status = PowerCommandStatus.STATUS_SUCCESS
        else:
            response.status = PowerCommandStatus.STATUS_INTERNAL_ERROR
        return response


@pytest.fixture
def simple_spot_wrapper(simple_spot: SpotFixture) -> Iterator[SpotWrapper]:
    # Use fixture address and port for wrapper.
    spot_wrapper = SpotWrapper(
        username="spot",
        password="spot",
        hostname=simple_spot.address,
        port=simple_spot.port,
        robot_name=simple_spot.api.name,
        cert_resource_glob=str(simple_spot.certificate_path),
        logger=logging.getLogger("spot"),
    )
    ok, message = spot_wrapper.claim()
    assert ok, message
    try:
        yield spot_wrapper
    finally:
        ok, message = spot_wrapper.release()
        assert ok, message


def test_spot_wrapper_setup(simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper) -> None:
    assert simple_spot_wrapper.is_valid
    assert simple_spot_wrapper.id is not None
    assert simple_spot_wrapper.id.nickname == simple_spot.api.name


def test_spot_wrapper_powering(simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper) -> None:
    assert not simple_spot_wrapper.check_is_powered_on()
    assert simple_spot_wrapper.toggle_power(True)
    assert simple_spot_wrapper.check_is_powered_on()

    power_state = simple_spot.api.robot_state.power_state
    assert power_state.motor_power_state == PowerState.MotorPowerState.MOTOR_POWER_STATE_ON


def test_spot_wrapper_commands(simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper) -> None:
    if simple_spot_wrapper.has_arm():
        # bosdyn.api.ManipulationApiService/ManipulationApi implementation is mocked
        # (as spot_wrapper.testing.mocks.MockSpot is automatically specified), so here
        # we provide a response in advance.
        request = ManipulationApiRequest()
        request.pick_object.frame_name = "gripper"
        request.pick_object.object_rt_frame.x = 1.0

        response = ManipulationApiResponse()
        response.manipulation_cmd_id = 1
        simple_spot.api.ManipulationApi.future.returns(response)

        ok, message, command_id = simple_spot_wrapper.manipulation_command(request)
        assert ok and response.manipulation_cmd_id == command_id, message

    # bosdyn.api.RobotCommandService/RobotCommand implementation is mocked
    # (as spot_wrapper.testing.mocks.MockSpot is automatically specified),
    # so here we provide the same response for every future command.
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response).repeatedly(2)
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_BEHAVIOR_FAULT
    simple_spot.api.RobotCommand.future.returns(response).forever()
    ok, message = simple_spot_wrapper.self_right()
    assert ok, message
    ok, message = simple_spot_wrapper.sit()
    assert ok, message
    ok, _ = simple_spot_wrapper.stand()
    assert not ok
    ok, _ = simple_spot_wrapper.sit()
    assert not ok


def test_mobility_command_feedback(simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper) -> None:
    non_processing_feedback_statuses: List[RobotCommandFeedbackStatus] = [
        RobotCommandFeedbackStatus.STATUS_UNKNOWN,
        RobotCommandFeedbackStatus.STATUS_COMMAND_OVERRIDDEN,
        RobotCommandFeedbackStatus.STATUS_COMMAND_TIMED_OUT,
        RobotCommandFeedbackStatus.STATUS_ROBOT_FROZEN,
        RobotCommandFeedbackStatus.STATUS_INCOMPATIBLE_HARDWARE,
    ]

    # bosdyn.api.RobotCommandService/RobotCommand implementation is mocked
    # (as spot_wrapper.testing.mocks.MockSpot is automatically specified),
    # so here we provide the same response for every future command.
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response).forever()

    def update_with_feedback_response(r: RobotCommandFeedbackResponse) -> None:
        simple_spot.api.RobotCommandFeedback.future.returns(r)
        simple_spot_wrapper.updateTasks()
        time.sleep(0.1)

    def assert_non_processing_stand_status(should_stand: bool) -> None:
        for status in non_processing_feedback_statuses:
            simple_spot_wrapper.stand()
            assert simple_spot_wrapper.last_stand_command is not None
            assert simple_spot_wrapper.is_standing == should_stand
            r = RobotCommandFeedbackResponse()
            r.feedback.synchronized_feedback.mobility_command_feedback.status = status
            update_with_feedback_response(r)
            assert simple_spot_wrapper.last_stand_command is None
            assert simple_spot_wrapper.is_standing == should_stand

    def assert_non_processing_sit_status(should_sit: bool) -> None:
        for status in non_processing_feedback_statuses:
            simple_spot_wrapper.sit()
            assert simple_spot_wrapper.last_sit_command is not None
            assert simple_spot_wrapper.is_sitting == should_sit
            r = RobotCommandFeedbackResponse()
            r.feedback.synchronized_feedback.mobility_command_feedback.status = status
            update_with_feedback_response(r)
            assert simple_spot_wrapper.last_sit_command is None
            assert simple_spot_wrapper.is_sitting == should_sit

    # initial standing state
    assert simple_spot_wrapper.last_stand_command is None
    assert not simple_spot_wrapper.is_standing

    # stand command with non-processing status - should reset the command and leave state unchanged
    assert_non_processing_stand_status(simple_spot_wrapper.is_standing)

    # stand command with processing status
    simple_spot_wrapper.stand()
    assert simple_spot_wrapper.last_stand_command is not None
    assert not simple_spot_wrapper.is_standing
    response = RobotCommandFeedbackResponse()
    response.feedback.synchronized_feedback.mobility_command_feedback.status = (
        RobotCommandFeedbackStatus.STATUS_PROCESSING
    )
    # command in progress
    response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status = (
        StandCommand.Feedback.STATUS_IN_PROGRESS
    )
    update_with_feedback_response(response)
    assert simple_spot_wrapper.last_stand_command is not None
    assert not simple_spot_wrapper.is_standing
    assert not simple_spot_wrapper.is_sitting
    # command finished
    response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status = (
        StandCommand.Feedback.STATUS_IS_STANDING
    )
    update_with_feedback_response(response)
    assert simple_spot_wrapper.last_stand_command is None
    assert simple_spot_wrapper.is_standing
    assert not simple_spot_wrapper.is_sitting

    # stand command with non-processing status - should reset the command and leave state unchanged
    assert_non_processing_stand_status(simple_spot_wrapper.is_standing)

    # initial sitting state
    assert simple_spot_wrapper.last_sit_command is None
    assert not simple_spot_wrapper.is_sitting

    # sit command with non-processing status - should reset the command and leave state unchanged
    assert_non_processing_sit_status(simple_spot_wrapper.is_sitting)

    # sit command with processing status
    simple_spot_wrapper.sit()
    assert simple_spot_wrapper.last_sit_command is not None
    assert not simple_spot_wrapper.is_sitting
    response = RobotCommandFeedbackResponse()
    response.feedback.synchronized_feedback.mobility_command_feedback.status = (
        RobotCommandFeedbackStatus.STATUS_PROCESSING
    )
    # command in progress
    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status = (
        SitCommand.Feedback.STATUS_IN_PROGRESS
    )
    update_with_feedback_response(response)
    assert simple_spot_wrapper.last_sit_command is not None
    assert not simple_spot_wrapper.is_sitting
    assert not simple_spot_wrapper.is_standing
    # command finished
    response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status = (
        SitCommand.Feedback.STATUS_IS_SITTING
    )
    update_with_feedback_response(response)
    assert simple_spot_wrapper.last_sit_command is None
    assert simple_spot_wrapper.is_sitting
    assert not simple_spot_wrapper.is_standing

    # sit command with non-processing status - should reset the command and leave state unchanged
    assert_non_processing_sit_status(simple_spot_wrapper.is_sitting)


def test_robot_movement_states(simple_spot: SpotFixture, simple_spot_wrapper: SpotWrapper) -> None:
    # bosdyn.api.RobotCommandService/RobotCommand implementation is mocked
    # (as spot_wrapper.testing.mocks.MockSpot is automatically specified),
    # so here we provide the same response for every future command.
    response = RobotCommandResponse()
    response.status = RobotCommandResponse.Status.STATUS_OK
    simple_spot.api.RobotCommand.future.returns(response).forever()

    assert simple_spot_wrapper.last_sit_command is None
    assert simple_spot_wrapper.last_stand_command is None
    assert simple_spot_wrapper.last_velocity_command_time is None
    assert simple_spot_wrapper.is_sitting
    assert not simple_spot_wrapper.is_standing
    assert not simple_spot_wrapper.is_moving

    simple_spot_wrapper.velocity_cmd(0, 0, 0)
    assert simple_spot_wrapper.last_velocity_command_time is not None
    simple_spot_wrapper.updateTasks()
    time.sleep(0.1)

    assert simple_spot_wrapper.last_sit_command is None
    assert simple_spot_wrapper.last_stand_command is None
    assert not simple_spot_wrapper.is_sitting
    assert simple_spot_wrapper.is_standing
    assert simple_spot_wrapper.is_moving
