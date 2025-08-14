# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

from spot_wrapper.testing.grpc import AutoServicer
from spot_wrapper.testing.mocks.auth import MockAuthService
from spot_wrapper.testing.mocks.cam import MockCAMService
from spot_wrapper.testing.mocks.directory import MockDirectoryService
from spot_wrapper.testing.mocks.estop import MockEStopService
from spot_wrapper.testing.mocks.keepalive import MockKeepaliveService
from spot_wrapper.testing.mocks.lease import MockLeaseService
from spot_wrapper.testing.mocks.license import MockLicenseService
from spot_wrapper.testing.mocks.payload import MockPayloadService
from spot_wrapper.testing.mocks.power import MockPowerService
from spot_wrapper.testing.mocks.robot_id import MockRobotIdService
from spot_wrapper.testing.mocks.robot_state import MockRobotStateService
from spot_wrapper.testing.mocks.time_sync import MockTimeSyncService
from spot_wrapper.testing.services import BaseSpotServicer


class BaseMockSpot(AutoServicer, BaseSpotServicer):
    """Base Spot mock."""

    name = "mockie"
    autocomplete = True


class MockSpot(
    BaseMockSpot,
    MockAuthService,
    MockCAMService,
    MockDirectoryService,
    MockEStopService,
    MockKeepaliveService,
    MockLeaseService,
    MockLicenseService,
    MockPayloadService,
    MockPowerService,
    MockRobotIdService,
    MockRobotStateService,
    MockTimeSyncService,
):
    """
    Nominal Spot mock.

    It implements the bare minimum for Spot wrapper initialization.
    For the rest, it relies on automatic specification.
    """

    autospec = True
