"""
Global variables used for configuration. These will not change at runtime.
"""
from collections import namedtuple

SPOT_CLIENT_NAME = "ros_spot"
MAX_COMMAND_DURATION = 1e5

"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"

"""List of point cloud sources"""
point_cloud_sources = ["velodyne-point-cloud"]

"""List of hand image sources for asynchronous periodic query"""
HAND_IMAGE_SOURCES = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]

"""List of body image sources for periodic query"""
CAMERA_IMAGE_SOURCES = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]
DEPTH_IMAGE_SOURCES = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
]
DEPTH_REGISTERED_IMAGE_SOURCES = [
    "frontleft_depth_in_visual_frame",
    "frontright_depth_in_visual_frame",
    "right_depth_in_visual_frame",
    "left_depth_in_visual_frame",
    "back_depth_in_visual_frame",
]
ImageBundle = namedtuple(
    "ImageBundle", ["frontleft", "frontright", "left", "right", "back"]
)
