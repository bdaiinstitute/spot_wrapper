# Automatic Robotic Stereo Camera Calibration with Charuco Target

# Table of Contents

1. [***Overview***](#overview)
2. [***Quick Start***](#quick-start)
3. [***Check if you have a Legacy Charuco Board***](#check-if-you-have-a-legacy-charuco-board)
4. [***All Flags***](#all-flags)

# Overview

![side by side comparison](registration_qualitative_example.jpg)

This utility streamlines automatic
camera calibration to **solve for the intrinsic and extrinsic parameters for two or more
cameras mounted in a fixed pose relative to each other on a robot**
based off of moving the robot to view a Charuco target from different poses. You can also calculate the calibration from an existing dataset of photos or make a new dataset from a pre-defined list of poses (if available). Additionally, the new calibrations can be directly written to the robot.

The CLI tool's saving capability allows to store multiple unique calibration runs in one configuration file
with calibration metadata, to document related runs with different setups or parameters.

For more info, see ```calibration_util.py```, where the functions
```get_multiple_perspective_camera_calibration_dataset``` and ```multistereo_calibration_charuco```
do most of the heavy lifting.

> ***NOTE:*** For a more in-depth explanation of this calibration tool, see ![this document](oldREADME.md).

# Quick Start

## Full Calibration

To run through the full calibration script to calibrate your Spot's hand camera (depth to rgb, with rgb at default resolution), first set up your robot and charuco board like below

![spot eye in hand cal](spot_eye_in_hand_setup.jpg)

You can then run the following command:

> ***WARNING:*** The robot will move. Be sure the robot is sitting (not docked) in front of the charuco board and that no one else is holding the lease before beginning.

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET> 
```

To save the collected images, add the ```--save_data``` and ```--data_path``` flags

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET>\ 
--data_path <PATH/TO/DATA/FOLDER> --save_data
```

To also save the calculated calibrations, add the ```--result_path``` flag

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET>\ 
--data_path <PATH/TO/DATA/FOLDER> --save_data --result_path <PATH/TO/RESULTS/FILE.YAML>
```

To overwrite the robot's internal calibration with your newly calculated one, also add the ```--save_to_robot``` flag

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET>\ 
--data_path <PATH/TO/DATA/FOLDER> --save_data --result_path <PATH/TO/RESULTS/FILE.YAML> --save_to_robot
```

> ***NOTE:*** If you are using a legacy charuco board, add ```--legacy_charuco_pattern True```. If you're not sure, go to [Check if you have a Legacy Charuco Board](#check-if-you-have-a-legacy-charuco-board).

## Calibration with Existing Dataset

If you already have a folder of images to calibrate on, then add the ```--from_data``` flag and remove the ```--save_data``` flag. This will *not* move the robot.

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET>\ 
--data_path <PATH/TO/DATA/FOLDER> --from_data 
```

You can still optionally add the ```--result_path <PATH/TO/RESULTS/FILE.YAML>``` and/or ```--save_to_robot``` flags to save the calculated calibration to a yaml file and/or overwrite the robot's internally saved calibrations, respectively.

## Overwrite Robot Calibration with Existing Yaml

If you have previously run the calibration script and saved the results, you can send those directly to the robot to be saved on its internal hardware without needing to run the entire calibration script by including the ```--from_yaml``` and ```--data_path``` flags and ensuring the ```<PATH/TO/RESULTS/FILE.YAML>``` goes to the correct calibration yaml file.

```bash
python3 calibrate_spot_hand_camera_cli.py --ip <ROBOT_IP> -u <USER> -pw <SECRET>\ 
--from_yaml --data_path <PATH/TO/RESULTS/FILE.YAML> --save_to_robot
```

## All Flags

Below are all the various flags available to you to further fine-tune your calibration. You can also see them all in your terminal by running

```bash
python3 calibrate_spot_hand_camera_cli.py -h
```

```bash
options:
  -h, --help            show this help message and exit
  --stereo_pairs STEREO_PAIRS, -sp STEREO_PAIRS
                        Capture images returns a list of images. Stereo pairs correspond towhat
                        index in the list of images' corresponding camerato calibrate to what
                        camera. Say capture_images returns [rgb_img, depth_img]and you want to
                        register depth to rgb, then the desired stereo pairis "[(1,0)]". If you
                        want to register more than one pair, you can do it like "[(1, 0),
                        (2,0)]."Make sure to put the stereo pairs in quotes so bash doesn't
                        complain
  --legacy_charuco_pattern LEGACY_CHARUCO_PATTERN, -l LEGACY_CHARUCO_PATTERN
                        Whether to use the legacy charuco pattern. For Spot Default board, this
                        should be True.If you aren't sure if your board is legacy, try supplying
                        the --check_board_patternarg to verify that the cv2 board matches your
                        board.
  --check_board_pattern
                        Whether to visually verify the board pattern (to check legacy and internal
                        corner ordering.
  --allow_default_internal_corner_ordering
                        Whether to allow default internal corner ordering. For new versions of
                        OpenCV, it is recommended to NOT set this parameter to make sure that
                        corners are ordered in a known pattern. Try supplying the
                        --check_board_pattern flag to check whether you should enable this flag
                        When checking, Z-Axis should point out of board.
  --photo_utilization_ratio PHOTO_UTILIZATION_RATIO, -pur PHOTO_UTILIZATION_RATIO
                        Photos that are collected/loaded vs. used for calibration are in a 1
                        tophoto utilization ratio. For getting a rough guess on cheaper
                        hardwarewithout losing collection fidelity. For example, set to 2 to only
                        use half the photos.
  --num_checkers_width NUM_CHECKERS_WIDTH, -ncw NUM_CHECKERS_WIDTH
                        How many checkers wide is your board
  --num_checkers_height NUM_CHECKERS_HEIGHT, -nch NUM_CHECKERS_HEIGHT
                        How many checkers tall is your board
  --dict_size {DICT_4X4_50,DICT_4X4_100,DICT_4X4_250,DICT_4X4_1000,DICT_5X5_50,DICT_5X5_100,DICT_5X5_250,DICT_5X5_1000,DICT_6X6_50,DICT_6X6_100,DICT_6X6_250,DICT_6X6_1000,DICT_7X7_50,DICT_7X7_100,DICT_7X7_250,DICT_7X7_1000,DICT_ARUCO_ORIGINAL}
                        Choose the ArUco dictionary size.
  --checker_dim CHECKER_DIM, -cd CHECKER_DIM
                        Checker size in meters
  --marker_dim MARKER_DIM, -md MARKER_DIM
                        Aruco Marker size in meters
  --data_path DATA_PATH, -dp DATA_PATH
                        The path in which to save images
  --result_path RESULT_PATH, -rp RESULT_PATH
                        Where to store calibration result as yaml file
  --tag TAG, -t TAG     What heading to put for the calibration in the config file.If this is your
                        first time running, the tag should be set to default for the sake of
                        interoperability with other functionality.If this is a shared config file
                        with other people, perhaps puta unique identifier, or default, if you'd
                        like to overridefor everyone.
  --unsafe_tag_save     If set, skips safety checks for tagging calibration.
  --dist_from_board_viewpoint_range DIST_FROM_BOARD_VIEWPOINT_RANGE [DIST_FROM_BOARD_VIEWPOINT_RANGE ...], -dfbvr DIST_FROM_BOARD_VIEWPOINT_RANGE [DIST_FROM_BOARD_VIEWPOINT_RANGE ...]
                        What distances to conduct calibrations at relative to the board. (along the
                        normal vector) Three value array arg defines the [Start, Stop), step. for
                        the viewpoint sweep. These are used to sample viewpoints for automatic
                        collection.
  --degrees, -d         Use degrees for rotation ranges (default)
  --radians, -r         Use radians for rotation ranges
  --x_axis_rot_viewpoint_range X_AXIS_ROT_VIEWPOINT_RANGE [X_AXIS_ROT_VIEWPOINT_RANGE ...], -xarvr X_AXIS_ROT_VIEWPOINT_RANGE [X_AXIS_ROT_VIEWPOINT_RANGE ...]
                        What range of viewpoints around x-axis to sample relative to boards normal
                        vector. Three value array arg defines the [Start, Stop), step. for the
                        viewpoint sweep These are used to sample viewpoints for automatic
                        collection. Assuming that the camera pose is in opencv/ROS format.
  --y_axis_rot_viewpoint_range Y_AXIS_ROT_VIEWPOINT_RANGE [Y_AXIS_ROT_VIEWPOINT_RANGE ...], -yarvr Y_AXIS_ROT_VIEWPOINT_RANGE [Y_AXIS_ROT_VIEWPOINT_RANGE ...]
                        What range of viewpoints around y-axis to sample relative to boards normal
                        vector. Three value array arg defines the [Start, Stop), step. for the
                        viewpoint sweep These are used to sample viewpoints for automatic
                        collection. Assuming that the camera pose is in opencv/ROS format.
  --z_axis_rot_viewpoint_range Z_AXIS_ROT_VIEWPOINT_RANGE [Z_AXIS_ROT_VIEWPOINT_RANGE ...], -zarvr Z_AXIS_ROT_VIEWPOINT_RANGE [Z_AXIS_ROT_VIEWPOINT_RANGE ...]
                        What range of viewpoints around z-axis to sample relative to boards normal
                        vector. Three value array arg defines the [Start, Stop), step. for the
                        viewpoint sweep These are used to sample viewpoints for automatic
                        collection. Assuming that the camera pose is in opencv/ROS format.
  --max_num_images MAX_NUM_IMAGES
                        The maximum number of images
  --settle_time SETTLE_TIME, -st SETTLE_TIME
                        How long to wait after movement to take a picture; don't want motion blur
  --save_data, -sd      whether to save the images to file
  --from_data, -fd      Whether to only calibrate from recorded dataset on file.
  --save_to_robot, -send
                        Whether to save the calibration to the robot.
  --from_yaml, -yaml    Whether the data is from a yaml file. Use this and the '--from_data' and '
                        --send' args to send a previously saved calibration yaml to the robot
  --ip IP, -i IP, -ip IP
                        The IP address of the Robot to calibrate
  --user USERNAME, -u USERNAME, --username USERNAME
                        Robot Username
  --pass PASSWORD, -pw PASSWORD, --password PASSWORD
                        Robot Password
  --spot_rgb_photo_width SPOT_RGB_PHOTO_WIDTH, -dpw SPOT_RGB_PHOTO_WIDTH
                        What resolution use for Spot's RGB Hand Camera (width). Currently, only 640
                        and 1920 are supported
  --spot_rgb_photo_height SPOT_RGB_PHOTO_HEIGHT, -dph SPOT_RGB_PHOTO_HEIGHT
                        What resolution use for Spot's RGB Hand Camera (width). Currently, only 480
                        and 1080 are supported
```

# Check if you have a Legacy Charuco Board

You only need to do this if using an opencv version after ```4.7```(
check with```python3 -c "import cv2; print(cv2.__version__)"```)

Through using the CLI tool (```python3 calibrate_multistereo_cameras_with_charuco_cli.py -h```), you can check if you have a legacy board through visually comparing the generated drawn virtual board to your physical charuco board target. Some legacy boards have an aruco tag in the top
left corner, whether as some non-legacy boards have a checker in the top left corner.
Also, check to see that the aruco tags match between virtual and physical boards.
It is important that the virtual board matches the physical board, otherwise this calibration
will not work.

```
python3 calibrate_multistereo_cameras_with_charuco_cli.py --check_board_pattern --legacy_charuco_pattern t 
```

There should be an axis at the center of the board, where the Y axis (green)
points upwards, the X axis (red) points to the right, and the figure should be labelled
as Z-axis out of board. If it isn't then try without legacy (```--legacy_charuco_pattern f```).

If you are using the default Spot Calibation board, and there is an aruco marker
in the top left corner, then it legacy (so supply true argument to legacy.)
