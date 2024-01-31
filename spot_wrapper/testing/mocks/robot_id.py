# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import grpc
from bosdyn.api.robot_id_pb2 import RobotIdRequest, RobotIdResponse
from bosdyn.api.robot_id_service_pb2_grpc import RobotIdServiceServicer


class MockRobotIdService(RobotIdServiceServicer):
    """A mock Spot robot id service."""

    def GetRobotId(self, request: RobotIdRequest, context: grpc.ServicerContext) -> RobotIdResponse:
        response = RobotIdResponse()
        response.robot_id.serial_number = "1234567890"
        response.robot_id.species = "mock-spot"
        response.robot_id.version = "0.0.0"
        response.robot_id.nickname = self.name
        response.robot_id.computer_serial_number = "1234567890ABCDEF"
        response.robot_id.software_release.version.major_version = 0
        response.robot_id.software_release.version.minor_version = 0
        response.robot_id.software_release.version.patch_level = 0
        return response
