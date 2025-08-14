# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import grpc
from bosdyn.api.time_sync_pb2 import (
    TimeSyncState,
    TimeSyncUpdateRequest,
    TimeSyncUpdateResponse,
)
from bosdyn.api.time_sync_service_pb2_grpc import TimeSyncServiceServicer


class MockTimeSyncService(TimeSyncServiceServicer):
    """
    A mock Spot time sync service.

    Always perfect clock synchronization.
    """

    def TimeSyncUpdate(self, request: TimeSyncUpdateRequest, context: grpc.ServicerContext) -> TimeSyncUpdateResponse:
        response = TimeSyncUpdateResponse()
        response.state.status = TimeSyncState.STATUS_OK
        response.state.best_estimate.clock_skew.seconds = 0
        response.state.best_estimate.clock_skew.nanos = 0
        response.clock_identifier = request.clock_identifier or "mock-clock"
        return response
