# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

from bosdyn.api.arm_surface_contact_service_pb2_grpc import (
    ArmSurfaceContactServiceServicer,
)
from bosdyn.api.auth_service_pb2_grpc import AuthServiceServicer
from bosdyn.api.auto_return.auto_return_service_pb2_grpc import (
    AutoReturnServiceServicer,
)
from bosdyn.api.autowalk.autowalk_service_pb2_grpc import AutowalkServiceServicer
from bosdyn.api.data_acquisition_plugin_service_pb2_grpc import (
    DataAcquisitionPluginServiceServicer,
)
from bosdyn.api.data_acquisition_store_service_pb2_grpc import (
    DataAcquisitionStoreServiceServicer,
)
from bosdyn.api.data_buffer_service_pb2_grpc import DataBufferServiceServicer
from bosdyn.api.data_service_pb2_grpc import DataServiceServicer
from bosdyn.api.directory_registration_service_pb2_grpc import (
    DirectoryRegistrationServiceServicer,
)
from bosdyn.api.directory_service_pb2_grpc import DirectoryServiceServicer
from bosdyn.api.docking.docking_service_pb2_grpc import DockingServiceServicer
from bosdyn.api.estop_service_pb2_grpc import EstopServiceServicer
from bosdyn.api.fault_service_pb2_grpc import FaultServiceServicer
from bosdyn.api.graph_nav.area_callback_service_pb2_grpc import (
    AreaCallbackServiceServicer,
)
from bosdyn.api.graph_nav.graph_nav_service_pb2_grpc import GraphNavServiceServicer
from bosdyn.api.graph_nav.map_processing_service_pb2_grpc import (
    MapProcessingServiceServicer,
)
from bosdyn.api.graph_nav.recording_service_pb2_grpc import (
    GraphNavRecordingServiceServicer,
)
from bosdyn.api.gripper_camera_param_service_pb2_grpc import (
    GripperCameraParamServiceServicer,
)
from bosdyn.api.image_service_pb2_grpc import ImageServiceServicer
from bosdyn.api.ir_enable_disable_service_pb2_grpc import IREnableDisableServiceServicer
from bosdyn.api.keepalive.keepalive_service_pb2_grpc import KeepaliveServiceServicer
from bosdyn.api.lease_service_pb2_grpc import LeaseServiceServicer
from bosdyn.api.license_service_pb2_grpc import LicenseServiceServicer
from bosdyn.api.local_grid_service_pb2_grpc import LocalGridServiceServicer
from bosdyn.api.log_status.log_status_service_pb2_grpc import LogStatusServiceServicer
from bosdyn.api.manipulation_api_service_pb2_grpc import ManipulationApiServiceServicer
from bosdyn.api.mission.mission_service_pb2_grpc import MissionServiceServicer
from bosdyn.api.mission.remote_service_pb2_grpc import RemoteMissionServiceServicer
from bosdyn.api.network_compute_bridge_service_pb2_grpc import (
    NetworkComputeBridgeWorkerServicer,
)
from bosdyn.api.payload_registration_service_pb2_grpc import (
    PayloadRegistrationServiceServicer,
)
from bosdyn.api.payload_service_pb2_grpc import PayloadServiceServicer
from bosdyn.api.point_cloud_service_pb2_grpc import PointCloudServiceServicer
from bosdyn.api.power_service_pb2_grpc import (
    PowerServiceServicer as PowerCommandServiceServicer,
)
from bosdyn.api.robot_command_service_pb2_grpc import (
    RobotCommandServiceServicer,
    RobotCommandStreamingServiceServicer,
)
from bosdyn.api.robot_id_service_pb2_grpc import RobotIdServiceServicer
from bosdyn.api.robot_state_service_pb2_grpc import (
    RobotStateServiceServicer,
    RobotStateStreamingServiceServicer,
)
from bosdyn.api.spot.choreography_service_pb2_grpc import ChoreographyServiceServicer
from bosdyn.api.spot.door_service_pb2_grpc import DoorServiceServicer
from bosdyn.api.spot.inverse_kinematics_service_pb2_grpc import InverseKinematicsServiceServicer
from bosdyn.api.spot.spot_check_service_pb2_grpc import SpotCheckServiceServicer
from bosdyn.api.spot_cam.service_pb2_grpc import (
    AudioServiceServicer,
    CompositorServiceServicer,
    HealthServiceServicer,
    LightingServiceServicer,
    MediaLogServiceServicer,
    NetworkServiceServicer,
    PowerServiceServicer,
    PtzServiceServicer,
    StreamQualityServiceServicer,
    VersionServiceServicer,
)
from bosdyn.api.time_sync_service_pb2_grpc import TimeSyncServiceServicer
from bosdyn.api.world_object_service_pb2_grpc import WorldObjectServiceServicer


class BaseSpotServicer(
    AreaCallbackServiceServicer,
    ArmSurfaceContactServiceServicer,
    AudioServiceServicer,
    AuthServiceServicer,
    AutoReturnServiceServicer,
    AutowalkServiceServicer,
    ChoreographyServiceServicer,
    CompositorServiceServicer,
    DataAcquisitionPluginServiceServicer,
    DataAcquisitionStoreServiceServicer,
    DataBufferServiceServicer,
    DataServiceServicer,
    DirectoryRegistrationServiceServicer,
    DirectoryServiceServicer,
    DockingServiceServicer,
    DoorServiceServicer,
    EstopServiceServicer,
    FaultServiceServicer,
    GraphNavRecordingServiceServicer,
    GraphNavServiceServicer,
    GripperCameraParamServiceServicer,
    HealthServiceServicer,
    IREnableDisableServiceServicer,
    ImageServiceServicer,
    InverseKinematicsServiceServicer,
    KeepaliveServiceServicer,
    LeaseServiceServicer,
    LicenseServiceServicer,
    LightingServiceServicer,
    LocalGridServiceServicer,
    LogStatusServiceServicer,
    ManipulationApiServiceServicer,
    MapProcessingServiceServicer,
    MediaLogServiceServicer,
    MissionServiceServicer,
    NetworkComputeBridgeWorkerServicer,
    NetworkServiceServicer,
    PayloadRegistrationServiceServicer,
    PayloadServiceServicer,
    PointCloudServiceServicer,
    PowerCommandServiceServicer,
    PowerServiceServicer,
    PtzServiceServicer,
    RemoteMissionServiceServicer,
    RobotCommandServiceServicer,
    RobotCommandStreamingServiceServicer,
    RobotIdServiceServicer,
    RobotStateServiceServicer,
    RobotStateStreamingServiceServicer,
    SpotCheckServiceServicer,
    StreamQualityServiceServicer,
    TimeSyncServiceServicer,
    VersionServiceServicer,
    WorldObjectServiceServicer,
):
    """
    Base Spot services servicer.

    It implements nothing.
    """

    name: str
