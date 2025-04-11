# Copyright (c) 2025 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import concurrent.futures
import math
from typing import Iterator

import grpc
import pytest
from bosdyn.api.autowalk.autowalk_service_pb2_grpc import AutowalkServiceStub
from bosdyn.api.data_chunk_pb2 import DataChunk
from bosdyn.api.header_pb2 import CommonError
from bosdyn.api.robot_command_pb2 import (
    JointControlStreamRequest,
    JointControlStreamResponse,
    RobotCommandRequest,
    RobotCommandResponse,
)
from bosdyn.api.robot_command_service_pb2_grpc import RobotCommandServiceStub, RobotCommandStreamingServiceStub
from bosdyn.api.robot_state_pb2 import RobotStateStreamRequest, RobotStateStreamResponse
from bosdyn.api.robot_state_service_pb2_grpc import RobotStateStreamingServiceStub

from spot_wrapper.testing.grpc import AutoServicer
from spot_wrapper.testing.services import BaseSpotServicer

PORT = 50051


class AutoSpotServicer(AutoServicer, BaseSpotServicer):
    autospec = True
    autotrack = True
    autocomplete = True


@pytest.fixture
def auto_spot_servicer() -> Iterator[AutoSpotServicer]:
    with concurrent.futures.ThreadPoolExecutor() as thread_pool:
        server = grpc.server(thread_pool)
        server.add_insecure_port(f"localhost:{PORT}")
        with AutoSpotServicer() as servicer:
            servicer.add_to(server)
            server.start()
            try:
                yield servicer
            finally:
                server.stop(grace=None)


@pytest.fixture
def auto_spot_channel(auto_spot_servicer: AutoSpotServicer) -> Iterator[grpc.Channel]:
    with grpc.insecure_channel(f"localhost:{PORT}") as channel:
        yield channel


def test_unary_unary_auto_service(auto_spot_servicer: AutoSpotServicer, auto_spot_channel: grpc.Channel) -> None:
    stub = RobotCommandServiceStub(auto_spot_channel)
    expected_response = RobotCommandResponse()
    expected_response.status = RobotCommandResponse.Status.STATUS_OK
    auto_spot_servicer.RobotCommand.future.returns(expected_response)
    expected_request = RobotCommandRequest()
    expected_request.header.client_name = "test"
    expected_request.command.full_body_command.stop_request.SetInParent()
    response = stub.RobotCommand(expected_request)
    assert auto_spot_servicer.RobotCommand.num_calls == 1
    request = auto_spot_servicer.RobotCommand.requests[0]
    assert request.command.full_body_command.HasField("stop_request")
    assert response.header.request_header.client_name == request.header.client_name
    assert response.header.error.code == CommonError.Code.CODE_OK
    assert response.status == expected_response.status


def test_unary_stream_auto_service(auto_spot_servicer: AutoSpotServicer, auto_spot_channel: grpc.Channel) -> None:
    stub = RobotStateStreamingServiceStub(auto_spot_channel)
    expected_response_stream: list[RobotStateStreamResponse] = []
    for i in range(10):
        response = RobotStateStreamResponse()
        response.joint_states.position.append(i)
        expected_response_stream.append(response)
    auto_spot_servicer.GetRobotStateStream.future.returns(expected_response_stream)
    expected_request = RobotStateStreamRequest()
    expected_request.header.client_name = "test"
    response_stream = list(stub.GetRobotStateStream(expected_request))
    for i, (expected_response, response) in enumerate(zip(expected_response_stream, response_stream)):
        assert response.header.request_header.client_name == expected_request.header.client_name
        assert response.header.error.code == CommonError.Code.CODE_OK
        assert (
            response.joint_states.position[0] == expected_response.joint_states.position[0]
        ), f"Bad joint position at index {i}"
    assert auto_spot_servicer.GetRobotStateStream.num_calls == 1
    request = auto_spot_servicer.GetRobotStateStream.requests[0]
    assert request.header.client_name == expected_request.header.client_name


def test_stream_unary_auto_service(auto_spot_servicer: AutoSpotServicer, auto_spot_channel: grpc.Channel) -> None:
    stub = RobotCommandStreamingServiceStub(auto_spot_channel)
    expected_response = JointControlStreamResponse()
    expected_response.status = JointControlStreamResponse.Status.STATUS_OK
    auto_spot_servicer.JointControlStream.future.returns(expected_response)
    expected_request_stream: list[JointControlStreamRequest] = []
    for i in range(10):
        request = JointControlStreamRequest()
        request.header.client_name = "test"
        request.joint_command.position.append(i * math.pi / 10)
        expected_request_stream.append(request)
    response = stub.JointControlStream(iter(expected_request_stream))
    assert auto_spot_servicer.JointControlStream.num_calls == 1
    request_stream = auto_spot_servicer.JointControlStream.requests[0]
    for i, (expected_request, request) in enumerate(zip(expected_request_stream, request_stream)):
        assert (
            expected_request.joint_command.position[0] == request.joint_command.position[0]
        ), f"Bad joint position command at index {i}"
    assert response.header.request_header.client_name == expected_request_stream[0].header.client_name
    assert response.header.error.code == CommonError.Code.CODE_OK
    assert response.status == expected_response.status


def test_stream_stream_auto_service(auto_spot_servicer: AutoSpotServicer, auto_spot_channel: grpc.Channel) -> None:
    stub = AutowalkServiceStub(auto_spot_channel)
    expected_request_stream = [DataChunk(total_size=i * 100) for i in range(10)]
    expected_response_stream = [DataChunk(total_size=i * 10) for i in range(10)]
    auto_spot_servicer.CompileAutowalk.future.returns(expected_response_stream)
    response_stream = list(stub.CompileAutowalk(iter(expected_request_stream)))
    for i, (expected_chunk, chunk) in enumerate(zip(expected_response_stream, response_stream)):
        assert expected_chunk.total_size == chunk.total_size, f"Bad response chunk at index {i}"
    assert auto_spot_servicer.CompileAutowalk.num_calls == 1
    request_stream = auto_spot_servicer.CompileAutowalk.requests[0]
    for i, (expected_chunk, chunk) in enumerate(zip(expected_request_stream, request_stream)):
        assert expected_chunk.total_size == chunk.total_size, f"Bad request chunk at index {i}"
