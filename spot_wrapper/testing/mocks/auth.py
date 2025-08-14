# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import grpc
from bosdyn.api.auth_pb2 import GetAuthTokenRequest, GetAuthTokenResponse
from bosdyn.api.auth_service_pb2_grpc import AuthServiceServicer


class MockAuthService(AuthServiceServicer):
    """A mock Spot authentication service."""

    def GetAuthToken(self, request: GetAuthTokenRequest, context: grpc.ServicerContext) -> GetAuthTokenResponse:
        response = GetAuthTokenResponse()
        response.status = GetAuthTokenResponse.Status.STATUS_OK
        response.token = "mock-token"
        return response
