"""
Global variables used for configuration. These will not change at runtime.
"""
from collections import namedtuple

SPOT_CLIENT_NAME = "ros_spot"
MAX_COMMAND_DURATION = 1e5

"""List of image sources for front image periodic query"""
front_image_sources = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "frontleft_depth",
    "frontright_depth",
]

"""List of image sources for side image periodic query"""
side_image_sources = [
    "left_fisheye_image",
    "right_fisheye_image",
    "left_depth",
    "right_depth",
]

"""List of image sources for rear image periodic query"""
rear_image_sources = ["back_fisheye_image", "back_depth"]

"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"

"""List of point cloud sources"""
point_cloud_sources = ["velodyne-point-cloud"]

"""List of image sources for hand image periodic query"""
hand_image_sources = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]

# TODO: Missing Hand images
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
