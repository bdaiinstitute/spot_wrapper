# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import typing

import grpc
import inflection
from bosdyn.api.directory_pb2 import (
    GetServiceEntryRequest,
    GetServiceEntryResponse,
    ListServiceEntriesRequest,
    ListServiceEntriesResponse,
    ServiceEntry,
)
from bosdyn.api.directory_service_pb2_grpc import DirectoryServiceServicer

from spot_wrapper.testing.grpc import collect_service_types


class MockDirectoryService(DirectoryServiceServicer):
    """
    A mock Spot directory service.

    By default it yields all services implemented by its class,
    using default authority and name conventions.
    """

    # Manage services with special names and/or authority.
    DEFAULT_SERVICES = {
        entry.type: entry
        for entry in [
            ServiceEntry(name="auth", type="bosdyn.api.AuthService", authority="auth.spot.robot"),
            ServiceEntry(
                name="payload-registration",
                type="bosdyn.api.RobotIdService",
                authority="id.spot.robot",
            ),
            ServiceEntry(
                name="payload-registration",
                type="bosdyn.api.PayloadRegistrationService",
                authority="payload-registration.spot.robot",
            ),
            ServiceEntry(
                name="robot-id",
                type="bosdyn.api.RobotIdService",
                authority="id.spot.robot",
            ),
            ServiceEntry(
                name="world-objects",
                type="bosdyn.api.WorldObjectService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="graph-nav-service",
                type="bosdyn.api.graph_nav.GraphNavService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="map-processing-service",
                type="bosdyn.api.graph_nav.MapProcessingService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="ray-cast",
                type="bosdyn.api.RayCastService",
                authority="ray-cast.spot.robot",
            ),
            ServiceEntry(
                name="autowalk-service",
                type="bosdyn.api.autowalk.AutowalkService",
                authority="ray-cast.spot.robot",
            ),
            ServiceEntry(
                name="manipulation",
                type="bosdyn.api.ManipulationApiService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-audio",
                type="bosdyn.api.spot_cam.AudioService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-compositor",
                type="bosdyn.api.spot_cam.CompositorService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-health",
                type="bosdyn.api.spot_cam.HealthService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-lighting",
                type="bosdyn.api.spot_cam.LightingService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-media-log",
                type="bosdyn.api.spot_cam.MediaLogService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-network",
                type="bosdyn.api.spot_cam.NetworkService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-power",
                type="bosdyn.api.spot_cam.PowerService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-ptz",
                type="bosdyn.api.spot_cam.PtzService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-stream-quality",
                type="bosdyn.api.spot_cam.StreamQualityService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="spot-cam-version",
                type="bosdyn.api.spot_cam.VersionService",
                authority="api.spot.robot",
            ),
            ServiceEntry(
                name="robot-mission",
                type="bosdyn.api.mission.MissionService",
                authority="api.spot.robot",
            ),
        ]
    }

    def __init__(
        self,
        *,
        services: typing.Optional[typing.Iterable[ServiceEntry]] = None,
        **kwargs: typing.Any,
    ) -> None:
        super().__init__(**kwargs)
        if services is None:
            services = []
            for service_type in collect_service_types(self):
                entry = ServiceEntry()
                if service_type not in self.DEFAULT_SERVICES:
                    _, _, service_typename = service_type.rpartition(".")
                    if service_typename.endswith("Service"):
                        service_name = service_typename[: -len("Service")]
                    service_name = inflection.underscore(service_name).replace("_", "-")
                    entry.type = service_type
                    entry.name = service_name
                    entry.authority = "api.spot.robot"
                else:
                    entry.CopyFrom(self.DEFAULT_SERVICES[service_type])
                services.append(entry)
        self._services = {entry.name: entry for entry in services}

    def GetServiceEntry(
        self, request: GetServiceEntryRequest, context: grpc.ServicerContext
    ) -> GetServiceEntryResponse:
        response = GetServiceEntryResponse()
        if request.service_name not in self._services:
            response.status = GetServiceEntryResponse.Status.STATUS_NONEXISTENT_SERVICE
            return response
        response.service_entry.CopyFrom(self._services[request.service_name])
        return response

    def ListServiceEntries(
        self, request: ListServiceEntriesRequest, context: grpc.ServicerContext
    ) -> ListServiceEntriesResponse:
        response = ListServiceEntriesResponse()
        response.service_entries.extend(self._services.values())
        return response
