# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import itertools
import typing

import grpc
from bosdyn.api.keepalive.keepalive_pb2 import (
    CheckInRequest,
    CheckInResponse,
    GetStatusRequest,
    GetStatusResponse,
    LivePolicy,
    ModifyPolicyRequest,
    ModifyPolicyResponse,
)
from bosdyn.api.keepalive.keepalive_service_pb2_grpc import KeepaliveServiceServicer


class MockKeepaliveService(KeepaliveServiceServicer):
    """
    A mock Spot keep-alive service.

    It bookkeeps all policies but enforces none.
    """

    def __init__(self, **kwargs: typing.Any) -> None:
        super().__init__(**kwargs)
        self._policy_ids = itertools.count()
        self._policies: typing.Dict[int, LivePolicy] = {}

    @property
    def policies(self) -> typing.Iterable[LivePolicy]:
        return self._policies.values()

    def ModifyPolicy(self, request: ModifyPolicyRequest, context: grpc.ServicerContext) -> ModifyPolicyResponse:
        response = ModifyPolicyResponse()

        for policy_id in request.policy_ids_to_remove:
            if policy_id not in self._policies:
                response.status = ModifyPolicyResponse.Status.STATUS_INVALID_POLICY_ID
                return response
            live_policy = self._policies[policy_id]
            response.removed_policies.append(live_policy)
            del self._policies[policy_id]

        if request.HasField("to_add"):
            live_policy = LivePolicy()
            live_policy.policy.CopyFrom(request.to_add)
            live_policy.policy_id = next(self._policy_ids)
            live_policy.client_name = request.header.client_name
            live_policy.last_checkin.CopyFrom(request.header.request_timestamp)
            self._policies[live_policy.policy_id] = live_policy
            response.added_policy.CopyFrom(live_policy)

        response.status = ModifyPolicyResponse.Status.STATUS_OK
        return response

    def CheckIn(self, request: CheckInRequest, context: grpc.ServicerContext) -> CheckInResponse:
        response = CheckInResponse()
        if request.policy_id not in self._policies:
            response.status = CheckInResponse.Status.STATUS_INVALID_POLICY_ID
            return response
        live_policy = self._policies[request.policy_id]
        live_policy.last_checkin.CopyFrom(request.header.request_timestamp)
        response.status = CheckInResponse.Status.STATUS_OK
        return response

    def GetStatus(self, request: GetStatusRequest, context: grpc.ServicerContext) -> GetStatusResponse:
        response = GetStatusResponse()
        response.status.extend(self._policies.values())
        return response
