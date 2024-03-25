# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import grpc
from bosdyn.api.lease_pb2 import (
    AcquireLeaseRequest,
    AcquireLeaseResponse,
    Lease,
    LeaseResource,
    LeaseUseResult,
    ListLeasesRequest,
    ListLeasesResponse,
    ResourceTree,
    RetainLeaseRequest,
    RetainLeaseResponse,
    ReturnLeaseRequest,
    ReturnLeaseResponse,
    TakeLeaseRequest,
    TakeLeaseResponse,
)
from bosdyn.api.lease_service_pb2_grpc import LeaseServiceServicer

from spot_wrapper.testing.helpers import walk_resource_tree


class MockLeaseService(LeaseServiceServicer):
    """
    A mock Spot lease service.

    By default it uses a minimal resource tree. Lease staling times are not enforced.
    """

    def __init__(
        self,
        *,
        resource_tree: typing.Optional[ResourceTree] = None,
        **kwargs: typing.Any,
    ) -> None:
        super().__init__(**kwargs)
        if resource_tree is None:
            resource_tree = ResourceTree()
            resource_tree.resource = "all-leases"
            subtree = resource_tree.sub_resources.add()
            subtree.resource = "body"
        self._resource_tree = resource_tree
        self._leasable_resources: typing.Dict[str, LeaseResource] = {}
        for i, resource in enumerate(walk_resource_tree(resource_tree)):
            leasable_resource = LeaseResource()
            leasable_resource.resource = resource.resource
            leasable_resource.lease.resource = resource.resource
            leasable_resource.lease.sequence.append(i)
            leasable_resource.lease.epoch = "mock-epoch"
            leasable_resource.is_stale = True
            self._leasable_resources[resource.resource] = leasable_resource
        self._latest_lease: typing.Optional[Lease] = None

    @property
    def leasable_resources(self) -> typing.Iterable[LeaseResource]:
        return list(self._leasable_resources.values())

    def AcquireLease(self, request: AcquireLeaseRequest, context: grpc.ServicerContext) -> AcquireLeaseResponse:
        response = AcquireLeaseResponse()
        if request.resource not in self._leasable_resources:
            response.status = AcquireLeaseResponse.Status.STATUS_INVALID_RESOURCE
            return response
        leasable_resource = self._leasable_resources[request.resource]
        if not leasable_resource.is_stale:
            response.status = AcquireLeaseResponse.Status.STATUS_RESOURCE_ALREADY_CLAIMED
            response.lease_owner.CopyFrom(leasable_resource.lease_owner)
            return response
        leasable_resource.lease.client_names.append(request.header.client_name)
        if self._latest_lease is None:
            self._latest_lease = Lease()
        self._latest_lease.CopyFrom(leasable_resource.lease)
        leasable_resource.lease_owner.client_name = request.header.client_name
        leasable_resource.is_stale = False
        response.lease.CopyFrom(leasable_resource.lease)
        response.lease_owner.CopyFrom(leasable_resource.lease_owner)
        response.status = AcquireLeaseResponse.Status.STATUS_OK
        return response

    def TakeLease(self, request: TakeLeaseRequest, context: grpc.ServicerContext) -> TakeLeaseResponse:
        response = TakeLeaseResponse()
        if request.resource not in self._leasable_resources:
            response.status = TakeLeaseResponse.Status.STATUS_INVALID_RESOURCE
            return response
        leasable_resource = self._leasable_resources[request.resource]
        leasable_resource.lease.client_names.append(request.header.client_name)
        if self._latest_lease is None:
            self._latest_lease = Lease()
        self._latest_lease.CopyFrom(leasable_resource.lease)
        leasable_resource.lease_owner.client_name = request.header.client_name
        leasable_resource.is_stale = False
        response.lease.CopyFrom(leasable_resource.lease)
        response.lease_owner.CopyFrom(leasable_resource.lease_owner)
        response.status = TakeLeaseResponse.Status.STATUS_OK
        return response

    def ReturnLease(self, request: ReturnLeaseRequest, context: grpc.ServicerContext) -> ReturnLeaseResponse:
        response = ReturnLeaseResponse()
        if request.lease.resource not in self._leasable_resources:
            response.status = ReturnLeaseResponse.Status.STATUS_INVALID_RESOURCE
            return response
        leasable_resource = self._leasable_resources[request.lease.resource]
        if leasable_resource.is_stale:
            response.status = ReturnLeaseResponse.Status.STATUS_NOT_ACTIVE_LEASE
            return response
        leasable_resource.is_stale = True
        leasable_resource.ClearField("lease_owner")
        response.status = ReturnLeaseResponse.Status.STATUS_OK
        return response

    def ListLeases(self, request: ListLeasesRequest, context: grpc.ServicerContext) -> ListLeasesResponse:
        response = ListLeasesResponse()
        response.resources.extend(self._leasable_resources.values())
        response.resource_tree.CopyFrom(self._resource_tree)
        return response

    def RetainLease(self, request: RetainLeaseRequest, context: grpc.ServicerContext) -> RetainLeaseResponse:
        response = RetainLeaseResponse()
        response.lease_use_result.attempted_lease.CopyFrom(request.lease)
        if self._latest_lease is not None:
            response.lease_use_result.latest_known_lease.CopyFrom(self._latest_lease)
        if request.lease.resource not in self._leasable_resources:
            response.lease_use_result.status = LeaseUseResult.Status.STATUS_INVALID_LEASE
            return response
        leasable_resource = self._leasable_resources[request.lease.resource]
        if leasable_resource.is_stale:
            response.lease_use_result.status = LeaseUseResult.Status.STATUS_REVOKED
            return response
        response.lease_use_result.owner.CopyFrom(leasable_resource.lease_owner)
        response.lease_use_result.status = LeaseUseResult.Status.STATUS_OK
        return response
