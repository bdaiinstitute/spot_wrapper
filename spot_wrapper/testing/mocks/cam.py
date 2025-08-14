# Copyright (c) 2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import grpc
from bosdyn.api.spot_cam.ptz_pb2 import ListPtzRequest, ListPtzResponse, PtzDescription
from bosdyn.api.spot_cam.service_pb2_grpc import PtzServiceServicer


class MockCAMService(PtzServiceServicer):
    """Mock Spot CAM services."""

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__(**kwargs)
        self.ptzs: typing.List[PtzDescription] = []

    def ListPtz(self, request: ListPtzRequest, context: grpc.ServicerContext) -> ListPtzResponse:
        response = ListPtzResponse()
        response.ptzs.extend(self.ptzs)
        return response
