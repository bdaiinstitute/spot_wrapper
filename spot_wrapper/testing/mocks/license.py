# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import grpc
from bosdyn.api.license_pb2 import (
    GetFeatureEnabledRequest,
    GetFeatureEnabledResponse,
    GetLicenseInfoRequest,
    GetLicenseInfoResponse,
    LicenseInfo,
)
from bosdyn.api.license_service_pb2_grpc import LicenseServiceServicer


class MockLicenseService(LicenseServiceServicer):
    """
    A mock Spot license service.

    It provides a license that never expires for all features.
    """

    def GetLicenseInfo(self, request: GetLicenseInfoRequest, context: grpc.ServicerContext) -> GetLicenseInfoResponse:
        response = GetLicenseInfoResponse()
        response.license.status = LicenseInfo.Status.STATUS_VALID
        response.license.id = "0123210"
        response.license.robot_serial = "1234567890"
        response.licensed_features.append("choreography")
        return response

    def GetFeatureEnabled(
        self, request: GetFeatureEnabledRequest, context: grpc.ServicerContext
    ) -> GetFeatureEnabledResponse:
        response = GetFeatureEnabledResponse()
        for feature_code in request.feature_codes:
            response.feature_enabled[feature_code] = True
        return response
