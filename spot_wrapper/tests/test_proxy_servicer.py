# Copyright (c) 2025 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import concurrent.futures
from typing import Iterator
from unittest.mock import Mock

import grpc
import pytest
from bosdyn.api.robot_command_pb2 import RobotCommandRequest, RobotCommandResponse
from bosdyn.api.robot_command_service_pb2_grpc import RobotCommandServiceStub
from bosdyn.api.robot_state_pb2 import RobotStateStreamRequest, RobotStateStreamResponse
from bosdyn.api.robot_state_service_pb2_grpc import RobotStateStreamingServiceStub

from spot_wrapper.testing.grpc import AutoServicer, BackServicer, ReverseProxyServicer
from spot_wrapper.testing.services import BaseSpotServicer

PORT = 50052


class ProxySpotServicer(AutoServicer, ReverseProxyServicer, BaseSpotServicer):
    ...


@pytest.fixture
def mock_spot_backbone() -> Mock:
    return Mock(BackServicer)


@pytest.fixture
def proxy_spot_servicer(mock_spot_backbone: BackServicer) -> Iterator[ProxySpotServicer]:
    with concurrent.futures.ThreadPoolExecutor() as thread_pool:
        server = grpc.server(thread_pool)
        server.add_insecure_port(f"localhost:{PORT}")
        servicer = ProxySpotServicer(server=mock_spot_backbone)
        servicer.add_to(server)
        server.start()
        try:
            yield servicer
        finally:
            server.stop(grace=None)


@pytest.fixture
def proxy_spot_channel(proxy_spot_servicer: ProxySpotServicer) -> Iterator[grpc.Channel]:
    with grpc.insecure_channel(f"localhost:{PORT}") as channel:
        yield channel


def test_unary_proxy_service(proxy_spot_channel: grpc.Channel, mock_spot_backbone: Mock) -> None:
    stub = RobotCommandServiceStub(proxy_spot_channel)
    expected_response = RobotCommandResponse()
    expected_response.status = RobotCommandResponse.Status.STATUS_OK
    future = concurrent.futures.Future[RobotCommandResponse]()
    future.set_result(expected_response)
    mock_spot_backbone.submit.return_value = future
    expected_request = RobotCommandRequest()
    expected_request.command.full_body_command.stop_request.SetInParent()
    response = stub.RobotCommand(expected_request)
    assert mock_spot_backbone.submit.call_count == 1
    request = mock_spot_backbone.submit.call_args.args[0]
    assert request.command.full_body_command.HasField("stop_request")
    assert response.status == expected_response.status


def test_stream_proxy_service(proxy_spot_channel: grpc.Channel, mock_spot_backbone: Mock) -> None:
    stub = RobotStateStreamingServiceStub(proxy_spot_channel)
    expected_response_stream: list[RobotStateStreamResponse] = []
    for i in range(10):
        response = RobotStateStreamResponse()
        response.joint_states.position.append(i)
        expected_response_stream.append(response)
    future = concurrent.futures.Future[list[RobotStateStreamResponse]]()
    future.set_result(expected_response_stream)
    mock_spot_backbone.submit.return_value = future
    expected_request = RobotStateStreamRequest()
    expected_request.header.client_name = "test"
    response_stream = stub.GetRobotStateStream(expected_request)
    for i, (expected_response, response) in enumerate(zip(expected_response_stream, response_stream)):
        assert (
            response.joint_states.position[0] == expected_response.joint_states.position[0]
        ), f"Bad joint position at index {i}"
    assert mock_spot_backbone.submit.call_count == 1
    request = mock_spot_backbone.submit.call_args.args[0]
    assert request.header.client_name == expected_request.header.client_name
