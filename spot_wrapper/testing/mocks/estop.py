# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import itertools
import typing

import grpc
from bosdyn.api.estop_pb2 import (
    DeregisterEstopEndpointRequest,
    DeregisterEstopEndpointResponse,
    EstopCheckInRequest,
    EstopCheckInResponse,
    EstopConfig,
    EstopStopLevel,
    EstopSystemStatus,
    GetEstopConfigRequest,
    GetEstopConfigResponse,
    GetEstopSystemStatusRequest,
    GetEstopSystemStatusResponse,
    RegisterEstopEndpointRequest,
    RegisterEstopEndpointResponse,
    SetEstopConfigRequest,
    SetEstopConfigResponse,
)
from bosdyn.api.estop_service_pb2_grpc import EstopServiceServicer
from bosdyn.api.header_pb2 import CommonError


class MockEStopService(EstopServiceServicer):
    """
    A mock Spot e-stop service.

    It provides a license that never expires for all features.
    """

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__(**kwargs)
        self._config_id_generator = (f"config#{i}" for i in itertools.count())
        self._endpoint_id_generator = (f"endpoint#{i}" for i in itertools.count())
        self.active_estop_configuration = EstopConfig()
        self.active_estop_configuration.unique_id = next(self._config_id_generator)
        self.estop_configurations = [self.active_estop_configuration]
        self.estop_status = EstopSystemStatus()
        self.estop_status.stop_level = EstopStopLevel.ESTOP_LEVEL_NONE

    def RegisterEstopEndpoint(
        self, request: RegisterEstopEndpointRequest, context: grpc.ServicerContext
    ) -> RegisterEstopEndpointResponse:
        response = RegisterEstopEndpointResponse()
        response.request.CopyFrom(request)
        estop_configurations = {config.unique_id: config for config in self.estop_configurations}
        if request.target_config_id not in estop_configurations:
            response.status = RegisterEstopEndpointResponse.Status.STATUS_CONFIG_MISMATCH
            return response
        estop_configuration = estop_configurations[request.target_config_id]
        if request.target_endpoint.unique_id:
            estop_endpoints = {ep.unique_id: ep for ep in estop_configuration.endpoints}
            if request.target_endpoint.unique_id not in estop_endpoints:
                response.status = RegisterEstopEndpointResponse.Status.STATUS_ENDPOINT_MISMATCH
                return response
            estop_endpoint = estop_endpoints[request.target_endpoint.unique_id]
        else:
            estop_endpoint = estop_configuration.endpoints.add()
            estop_endpoint.unique_id = next(self._endpoint_id_generator)
            endpoint_status = self.estop_status.endpoints.add()
            endpoint_status.endpoint.CopyFrom(estop_endpoint)
            endpoint_status.stop_level = EstopStopLevel.ESTOP_LEVEL_NONE
        unique_id = estop_endpoint.unique_id
        estop_endpoint.CopyFrom(request.new_endpoint)
        estop_endpoint.unique_id = unique_id
        response.new_endpoint.CopyFrom(estop_endpoint)
        response.status = RegisterEstopEndpointResponse.Status.STATUS_SUCCESS
        return response

    def DeregisterEstopEndpoint(
        self, request: DeregisterEstopEndpointRequest, context: grpc.ServicerContext
    ) -> DeregisterEstopEndpointResponse:
        response = DeregisterEstopEndpointResponse()
        response.request.CopyFrom(request)
        estop_configurations = {config.unique_id: config for config in self.estop_configurations}
        if request.target_config_id not in estop_configurations:
            response.status = RegisterEstopEndpointResponse.Status.STATUS_CONFIG_MISMATCH
            return response
        estop_configuration = estop_configurations[request.target_config_id]
        estop_endpoint_indices = {ep.unique_id: index for index, ep in enumerate(estop_configuration.endpoints)}
        if request.target_endpoint.unique_id not in estop_endpoint_indices:
            response.status = RegisterEstopEndpointResponse.Status.STATUS_ENDPOINT_MISMATCH
            return response
        del estop_configuration.endpoints[estop_endpoint_indices[request.target_endpoint.unique_id]]
        del self.estop_status.endpoints[estop_endpoint_indices[request.target_endpoint.unique_id]]
        self.estop_status.stop_level = min(ep.stop_level for ep in self.estop_status.endpoints)
        response.status = DeregisterEstopEndpointResponse.Status.STATUS_SUCCESS
        return response

    def EstopCheckIn(self, request: EstopCheckInRequest, context: grpc.ServicerContext) -> EstopCheckInResponse:
        response = EstopCheckInResponse()
        response.request.CopyFrom(request)
        response.challenge = (request.challenge or 1) + 1
        estop_endpoints = {ep.unique_id: ep for cfg in self.estop_configurations for ep in cfg.endpoints}
        if request.endpoint.unique_id not in estop_endpoints:
            response.status = EstopCheckInResponse.Status.STATUS_ENDPOINT_UNKNOWN
            return response
        endpoint_status = next(
            (
                endpoint_status
                for endpoint_status in self.estop_status.endpoints
                if endpoint_status.endpoint.unique_id == request.endpoint.unique_id
            ),
            None,
        )
        if endpoint_status is None:
            response.status = EstopCheckInResponse.Status.STATUS_ENDPOINT_UNKNOWN
            return response
        response.status = EstopCheckInResponse.Status.STATUS_OK
        endpoint_status.stop_level = request.stop_level
        self.estop_status.stop_level = min(ep.stop_level for ep in self.estop_status.endpoints)
        return response

    def GetEstopConfig(self, request: GetEstopConfigRequest, context: grpc.ServicerContext) -> GetEstopConfigResponse:
        response = GetEstopConfigResponse()
        response.request.CopyFrom(request)
        if not request.target_config_id:
            response.active_config.CopyFrom(self.active_estop_configuration)
            return response
        estop_configurations = {config.unique_id: config for config in self.estop_configurations}
        if request.target_config_id not in estop_configurations:
            response.header.error.code = CommonError.CODE_INVALID_REQUEST
            return response
        response.active_config.CopyFrom(estop_configurations[request.target_config_id])
        return response

    def SetEstopConfig(self, request: SetEstopConfigRequest, context: grpc.ServicerContext) -> SetEstopConfigResponse:
        response = SetEstopConfigResponse()
        response.request.CopyFrom(request)
        if request.target_config_id:
            estop_configurations = {config.unique_id: config for config in self.estop_configurations}
            if request.target_config_id not in estop_configurations:
                response.status = SetEstopConfigResponse.Status.STATUS_INVALID_ID
                return response
            self.active_estop_configuration = estop_configurations[request.target_config_id]
        else:
            self.active_estop_configuration = EstopConfig()
            self.active_estop_configuration.unique_id = next(self._config_id_generator)
            self.estop_configurations.append(self.active_estop_configuration)
        self.active_estop_configuration.CopyFrom(request.config)
        response.active_config.CopyFrom(self.active_estop_configuration)
        response.status = SetEstopConfigResponse.Status.STATUS_SUCCESS
        return response

    def GetEstopSystemStatus(
        self, request: GetEstopSystemStatusRequest, context: grpc.ServicerContext
    ) -> GetEstopSystemStatusResponse:
        response = GetEstopSystemStatusResponse()
        response.status.CopyFrom(self.estop_status)
        return response
