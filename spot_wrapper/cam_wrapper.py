import asyncio
import datetime
import enum
import os.path
import pathlib
import shutil
import threading
import typing
import wave
import time
import math

import bosdyn.client
import cv2
import numpy as np
from PIL import Image
from aiortc import RTCConfiguration
from bosdyn.api import image_pb2
from bosdyn.api.data_chunk_pb2 import DataChunk
from bosdyn.api.spot_cam import audio_pb2
from bosdyn.api.spot_cam.camera_pb2 import Camera
from bosdyn.api.spot_cam.logging_pb2 import Logpoint
from bosdyn.api.spot_cam.ptz_pb2 import PtzDescription, PtzVelocity, PtzPosition
from bosdyn.client import Robot
from bosdyn.client import spot_cam
from bosdyn.client.payload import PayloadClient
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.health import HealthClient
from bosdyn.client.spot_cam.lighting import LightingClient
from bosdyn.client.spot_cam.media_log import MediaLogClient
from bosdyn.client.spot_cam.power import PowerClient
from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.streamquality import StreamQualityClient

from spot_wrapper.cam_webrtc_client import WebRTCClient
from spot_wrapper.wrapper import SpotWrapper


class LightingWrapper:
    """
    Wrapper for LED brightness interaction
    """

    class LEDPosition(enum.Enum):
        """
        Values indicate the position of the specified LED in the brightness list
        """

        REAR_LEFT = 0
        FRONT_LEFT = 1
        FRONT_RIGHT = 2
        REAR_RIGHT = 3

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: LightingClient = robot.ensure_client(
            LightingClient.default_service_name
        )

    def set_led_brightness(self, brightness):
        """
        Set the brightness of the LEDs to the specified brightness

        Args:
            brightness: LEDs will all be set to this brightness, which should be in the range [0, 1]. The value will
                        be clipped if outside this range.
        """
        # Clamp brightness to [0,1] range
        brightness = min(max(brightness, 0), 1)
        self.client.set_led_brightness([brightness] * 4)

    def get_led_brightness(self) -> typing.List[float]:
        """
        Get the brightness of the LEDS

        Returns:
            List of floats indicating current brightness of each LED, in the order they are specified in the
            LEDPosition enum
        """
        return self.client.get_led_brightness()


class PowerWrapper:
    """
    Wrapper for power interaction
    """

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: PowerClient = robot.ensure_client(PowerClient.default_service_name)

    def get_power_status(self):
        """
        Get power status for the devices
        """
        return self.client.get_power_status()

    def set_power_status(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Set power status for each of the devices

        Args:
            ptz:
            aux1: ??
            aux2: ??
            external_mic:
        """
        self.client.set_power_status(ptz, aux1, aux2, external_mic)

    def cycle_power(
        self,
        ptz: typing.Optional[bool] = None,
        aux1: typing.Optional[bool] = None,
        aux2: typing.Optional[bool] = None,
        external_mic: typing.Optional[bool] = None,
    ):
        """
        Cycle power of the specified devices

        Args:
            ptz:
            aux1:
            aux2:
            external_mic:
        """
        self.client.cycle_power(ptz, aux1, aux2, external_mic)


class CompositorWrapper:
    """
    Wrapper for compositor interaction
    """

    def __init__(self, robot: Robot, logger):
        self.logger = logger
        self.client: CompositorClient = robot.ensure_client(
            CompositorClient.default_service_name
        )

    def list_screens(self) -> typing.List[str]:
        """
        List the available screens - this includes individual cameras and also panoramic and other stitched images
        provided by the camera

        Returns:
             List of strings indicating available screens
        """
        return [screen.name for screen in self.client.list_screens()]

    def get_visible_cameras(self):
        """
        Get the camera data for the camera currently visible on the stream

        Returns:
            List of visible camera streams
        """
        return self.client.get_visible_cameras()

    def set_screen(self, screen: str):
        """
        Set the screen to be streamed over the network

        Args:
            screen: Screen to show
        """
        self.client.set_screen(screen)

    def get_screen(self) -> str:
        """
        Get the screen currently being streamed over the network

        Returns:
            Name of the currently displayed screen
        """
        return self.client.get_screen()

    def set_ir_colormap(self, colormap, min_temp, max_temp, auto_scale=True):
        """
        Set the colormap used for the IR camera

        Args:
            colormap: Colormap to use, options are https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ircolormap-colormap
            min_temp: Minimum temperature on the scale
            max_temp: Maximum temperature on the scale
            auto_scale: Auto-scales the colormap according to the image. If this is set min_temp and max_temp are
                        ignored
        """
        self.client.set_ir_colormap(colormap, min_temp, max_temp, auto_scale)

    def set_ir_meter_overlay(self, x, y, enable=True):
        """
        Set the reticle position on the Spot CAM IR.

        Args:
            x: Horizontal coordinate between 0 and 1
            y: Vertical coordinate between 0 and 1
            enable: If true, enable the reticle on the display
        """
        self.client.set_ir_meter_overlay(x, y, enable)


class HealthWrapper:
    """
    Wrapper for health details
    """

    def __init__(self, robot, logger):
        self.client: HealthClient = robot.ensure_client(
            HealthClient.default_service_name
        )
        self.logger = logger

    def get_bit_status(
        self,
    ) -> typing.Tuple[typing.List[str], typing.List[typing.Tuple[int, str]]]:
        """
        Get fault events and degradations

        Returns:
            Dictionary

        """
        bit_status = self.client.get_bit_status()
        events = []
        for event in bit_status[0]:
            events.append(event)

        degradations = []
        for degradation in bit_status[1]:
            degradations.append((degradation.type, degradation.description))
        return events, degradations

    def get_temperature(self) -> typing.Tuple[str, float]:
        """
        Get temperatures of various components of the camera

        Returns:
            Tuple of string and float indicating the component and its temperature in celsius
        """
        return [
            (composite.channel_name, composite.temperature / 1e3)
            for composite in self.client.get_temperature()
        ]

    # def get_system_log(self):
    #     """
    #     This seems to always time out
    #     """
    #     return self.client.get_system_log()


class AudioWrapper:
    """
    Wrapper for audio commands on the camera
    """

    def __init__(self, robot, logger):
        self.client: AudioClient = robot.ensure_client(AudioClient.default_service_name)
        self.logger = logger

    def list_sounds(self) -> typing.List[str]:
        """
        List sounds available on the device

        Returns:
            List of names of available sounds
        """
        return self.client.list_sounds()

    def set_volume(self, percentage):
        """
        Set the volume at which sounds should be played

        Args:
            percentage: How loud sounds should be from 0 to 100%
        """
        self.client.set_volume(percentage)

    def get_volume(self):
        """
        Get the current volume at which sounds are played

        Returns:
            Current volume as a percentage
        """
        return self.client.get_volume()

    def play_sound(self, sound_name, gain=1.0):
        """
        Play a sound which is on the device

        Args:
            sound_name: Name of the sound to play
            gain: Volume gain multiplier
        """
        sound = audio_pb2.Sound(name=sound_name)
        self.client.play_sound(sound, gain)

    def load_sound(self, sound_file, name):
        """
        Load a sound from a wav file and save it with the given name onto the device
        Args:
            sound_file: Wav file to read from
            name: Name to assign to the sound

        Raises:
            IOError: If the given file is not a file
            wave.Error: If the given file is not a wav file
        """
        full_path = os.path.abspath(os.path.expanduser(sound_file))
        print(full_path)
        if not os.path.isfile(full_path):
            raise IOError(f"Tried to load sound from {full_path} but it is not a file.")

        sound = audio_pb2.Sound(name=name)

        with wave.open(full_path, "rb") as fh:
            # Use this to make sure that the file is actually a wav file
            pass

        with open(full_path, "rb") as fh:
            data = fh.read()

        self.client.load_sound(sound, data)

    def delete_sound(self, name):
        """
        Delete a sound from the device

        Args:
            name: Name of the sound to delete
        """
        self.client.delete_sound(audio_pb2.Sound(name=name))


class StreamQualityWrapper:
    """
    Wrapper for stream quality commands
    """

    def __init__(self, robot, logger):
        self.client: StreamQualityClient = robot.ensure_client(
            StreamQualityClient.default_service_name
        )
        self.logger = logger

    def set_stream_params(self, target_bitrate, refresh_interval, idr_interval, awb):
        """
        Set image compression and postprocessing parameters

        Note: It is currently not possible to turn off the auto white balance. You will get a crash

        Args:
            target_bitrate: Compression level target in bits per second
            refresh_interval: How often the whole feed should be refreshed (in frames)
            idr_interval: How often an IDR message should be sent (in frames)
            awb: Mode for automatic white balance
        """
        self.client.set_stream_params(target_bitrate, refresh_interval, idr_interval, 0)

    def get_stream_params(self) -> typing.Dict[str, int]:
        """
        Get the current stream quality parameters

        Returns:
            Dictionary containing the parameters
        """
        params = self.client.get_stream_params()
        param_dict = {
            "target_bitrate": params.targetbitrate.value,
            "refresh_interval": params.refreshinterval.value,
            "idr_interval": params.idrinterval.value,
            "awb": params.awb.awb,
        }

        return param_dict

    def enable_congestion_control(self, enable):
        """
        Enable congestion control on the receiver... not sure what this does

        Args:
            enable: If true, enable congestion control
        """
        self.client.enable_congestion_control(enable)


class SpotCamCamera(enum.Enum):
    """
    More obvious names for the spot cam cameras
    """

    PANORAMIC = "pano"
    PTZ = "ptz"
    IR = "ir"
    REAR_LEFT_PANO = "c0"
    FRONT_LEFT_PANO = "c1"
    FRONT_PANO = "c2"
    FRONT_RIGHT_PANO = "c3"
    REAR_RIGHT_PANO = "c4"


class MediaLogWrapper:
    """
    Wrapper for interacting with the media log. Allows saving of images from the camera to logpoints for full
    resolution high quality imaging. Importantly, also has information about the cameras available.

    Some functionality adapted from https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/spot_cam/media_log.py
    """

    def __init__(self, robot, logger) -> None:
        self.client: MediaLogClient = robot.ensure_client(
            MediaLogClient.default_service_name
        )
        self.logger = logger

    def list_cameras(self) -> typing.List[Camera]:
        """
        List the cameras on the spot cam
        """
        return self.client.list_cameras()

    def list_logpoints(self) -> typing.List[Logpoint]:
        """
        List logpoints stored on the camera

        Returns:
            List of logpoints
        """
        return self.client.list_logpoints()

    def retrieve_logpoint(
        self, name: str, raw: bool = False
    ) -> typing.Tuple[Logpoint, DataChunk]:
        """
        Retrieve a logpoint from the camera

        Args:
            name: Name of the logpoint to retrieve
            raw: If true, retrieve raw data from the logpoint. Generally used for IR? See
                 https://github.com/boston-dynamics/spot-sdk/blob/aa607fec2e32f880ad55da8bf186ce1b3384c891/python/examples/spot_cam/media_log.py#L168-L171

        Returns:
            Logpoint with the given name, or None if it doesn't exist
        """
        # TODO: What does this return if the logpoint doesn't exist? Just an empty logpoint?
        if raw:
            return self.client.retrieve_raw_data(logpoint=Logpoint(name=name))
        else:
            return self.client.retrieve(logpoint=Logpoint(name=name))

    def get_logpoint_status(self, name: str) -> Logpoint.LogStatus:
        """
        Get the status of the logpoint

        https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference.html?highlight=listlogpoints#logpoint-logstatus

        Args:
            name: Retrieve the status for the logpoint with this name

        Returns:
            Status of the logpoint
        """
        return self.client.get_status(logpoint=Logpoint(name=name))

    def delete_logpoint(self, name: str) -> None:
        """
        Delete a logpoint from the camera

        Args:
            name: Delete this logpoint
        """
        self.client.delete(logpoint=Logpoint(name=name))

    def store(
        self, camera: SpotCamCamera, tag: typing.Optional[str] = None
    ) -> Logpoint:
        """
        Take a snapshot of the data currently on the given camera and store it to a logpoint.

        Args:
            camera: Camera for which a logpoint should be recorded
            tag: Optional tag to associate with the logpoint

        Returns:
            Logpoint containing information about the stored data
        """
        return self.client.store(
            camera=Camera(name=camera.value), record_type=Logpoint.STILLIMAGE, tag=tag
        )

    def tag(self, name: str, tag: str) -> None:
        """
        Update the tag of the given logpoint

        Args:
            name: Name of the Logpoint for which the tag should be updated
            tag: New tag for the Logpoint
        """
        self.client.tag(logpoint=Logpoint(name=name, tag=tag))

    def _build_filename(
        self,
        logpoint: Logpoint,
        filename: str,
        extension: str,
        camera: typing.Optional[SpotCamCamera] = None,
    ) -> str:
        """
        Build a filename from a base name. Returns files of the form basefilename_cameraname_iso_date


        Args:
            logpoint: Build a filename for this logpoint
            filename: Base filename to use
            extension: Extension to use
            camera: If provided, the camera name will be added to the full filename

        Returns:
            Filename of the form filename{_cameraname}_YYYY-mm-ddTHH:MM:SS.MS.extension
        """
        if not extension.startswith("."):
            extension = f".{extension}"

        log_time = datetime.datetime.fromtimestamp(logpoint.timestamp.seconds)
        log_time.replace(microsecond=int(logpoint.timestamp.nanos * 0.001))
        if camera:
            return f"{filename}_{camera.name}_{log_time.isoformat()}{extension}"
        else:
            return f"{filename}_{log_time.isoformat()}{extension}"

    def save_logpoint_image(
        self,
        logpoint_name: str,
        path: str,
        base_filename: str,
        raw: bool = False,
        camera: typing.Optional[SpotCamCamera] = None,
        use_rgb24: bool = False,
    ) -> typing.Optional[str]:
        """
        Save the data in a logpoint to the given file on the caller's local machine.

        Adapted from https://github.com/boston-dynamics/spot-sdk/blob/aa607fec2e32f880ad55da8bf186ce1b3384c891/python/examples/spot_cam/media_log.py#L166-L206

        Args:
            logpoint_name: Save the image associated with this logpoint
            path: Save the data to this directory
            base_filename: Use this filename as the base name for the image file
            raw: If true, retrieve raw data rather than processed data. Useful for IR images?
            camera: If set, add the name of the camera to the output filename. The logpoint doesn't store this information
            use_rgb24: If set, save the ptz image in .rgb24 format without compression. By default it is saved to png

        Returns:
            Filename the image was saved to, or None if saving failed
        """

        logpoint, image = self.retrieve_logpoint(logpoint_name, raw=raw)

        save_path = os.path.abspath(os.path.expanduser(path))

        if not os.path.isdir(save_path):
            pathlib.Path(save_path).mkdir(parents=True, exist_ok=True)

        # Special case for 16 bit raw thermal image
        if logpoint.image_params.format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            np_img = np.frombuffer(image, dtype=np.uint16).byteswap()
            np_img = np_img.reshape(
                (logpoint.image_params.height, logpoint.image_params.width, 1)
            )
            full_path = os.path.join(
                save_path,
                self._build_filename(logpoint, base_filename, ".pgm", SpotCamCamera.IR),
            )
            cv2.imwrite(full_path, np_img)
            return full_path

        # Pano and IR both come in as JPEG from retrieve command
        if (
            logpoint.image_params.height == 4800
            or logpoint.image_params.height == 2400
            or (
                logpoint.image_params.width == 640
                and logpoint.image_params.height == 512
            )
        ):
            full_path = os.path.join(
                save_path,
                self._build_filename(logpoint, base_filename, ".jpg", camera),
            )
            # If we're saving as jpg we don't want to rewrite the image file with use_rgb24
            jpg = True
        else:
            full_path = os.path.join(
                save_path,
                self._build_filename(logpoint, base_filename, ".rgb24", camera),
            )
            jpg = False

        # The original method saves the image to file first, then reads and saves it in a different way if rgb24 is
        # not requested, so we do it the same way. There's probably a better way to do it.  Maybe frombytes with the
        # raw option?
        with open(full_path, "wb") as f:
            f.write(image)

        if not jpg and not use_rgb24:
            with open(full_path, mode="rb") as fd:
                data = fd.read()

            mode = "RGB"
            try:
                image = Image.frombuffer(
                    mode,
                    (logpoint.image_params.width, logpoint.image_params.height),
                    data,
                    "raw",
                    mode,
                    0,
                    1,
                )
            except ValueError as e:
                self.logger.error(
                    f"Error while trying to save image as png: {e}. You may need to restart the camera to fix this."
                )
                return None

            os.remove(full_path)  # remove the rgb24 image
            full_path = os.path.join(
                save_path,
                self._build_filename(logpoint, base_filename, ".png", camera),
            )
            image.save(full_path)

        return logpoint

    def store_and_save_image(
        self, camera: SpotCamCamera, path: str, base_filename: str, delete: bool = True
    ) -> typing.Optional[str]:
        """
        Stores a logpoint and saves the data to the given path on the caller's local machine. Blocks until the image
        is ready to be saved.

        Args:
            camera: Camera from which data should be saved
            path: Save the data to this directory
            base_filename: Use this as the base name of the saved file
            delete: If true, delete the logpoint which is created after saving the image

        Returns:
            File the image was saved to, or None if saving failed
        """
        logpoint = self.store(camera)
        # Wait until the data is stored on disk before attempting a save
        while self.get_logpoint_status(logpoint.name).status != Logpoint.COMPLETE:
            logpoint = self.get_logpoint_status(logpoint.name)

        output_fname = self.save_logpoint_image(
            logpoint.name,
            path,
            base_filename,
            raw=False,
            camera=camera,
        )

        if delete:
            # Since we have saved the logpoint image, can probably safely delete it
            self.delete_logpoint(logpoint.name)

        return output_fname


class PTZWrapper:
    """
    Wrapper for controlling the PTZ unit
    """

    def __init__(self, robot, logger):
        self.client: PtzClient = robot.ensure_client(PtzClient.default_service_name)
        self.logger = logger
        self.ptzs = {}
        descriptions = self.client.list_ptz()
        for description in descriptions:
            self.ptzs[description.name] = description

    def list_ptz(self) -> typing.Dict[str, typing.Dict]:
        """
        List the available ptz units on the device

        Returns:
            Dict of descriptions of ptz units
        """
        ptzs = []

        descriptions = self.client.list_ptz()
        for ptz_desc in descriptions:
            ptzs.append(ptz_desc)
            # Also update the internal list of raw ptz definitions
            self.ptzs[ptz_desc.name] = ptz_desc

        return ptzs

    def _get_ptz_description(self, name):
        """
        Get the bosdyn version of the ptz description

        Args:
            name: Get description for this ptz

        Returns:
            PtzDescription
        """
        if name not in self.ptzs:
            self.logger.warn(
                f"Tried to retrieve description for ptz {name} but it does not exist."
            )
            return None

        return self.ptzs[name]

    def _clamp_value_to_limits(self, value, limits: PtzDescription.Limits):
        """
        Clamp the given value to the specified limits. If the limits are unspecified (i.e. both 0), the value is not
        clamped

        Args:
            value: Value to clamp
            limits: PTZ description limit proto

        Returns:
            Value clamped to limits
        """
        if limits.max.value == 0 and limits.min.value == 0:
            # If both max and min are zero, this means the limit is unset. The documentation states that if a limit
            # is unset, then all positions are valid.
            # https://dev.bostondynamics.com/protos/bosdyn/api/proto_reference#ptzdescription
            return value

        return max(min(value, limits.max.value), limits.min.value)

    def _clamp_request_to_limits(
        self, ptz_name, pan, tilt, zoom
    ) -> typing.Tuple[float, float, float]:
        """

        Args:
            ptz_name: Name of the ptz for which the pan, tilt, and zoom should be clamped

        Returns:
            Tuple of pan, tilt, zoom, clamped to the limits of the requested ptz
        """
        ptz_desc = self._get_ptz_description(ptz_name)

        return (
            self._clamp_value_to_limits(pan, ptz_desc.pan_limit),
            self._clamp_value_to_limits(tilt, ptz_desc.tilt_limit),
            self._clamp_value_to_limits(zoom, ptz_desc.zoom_limit),
        )

    def get_ptz_position(self, ptz_name) -> PtzPosition:
        """
        Get the position of the ptz with the given name

        Args:
            ptz_name: Name of the ptz

        Returns:
            ptz position proto
        """
        return self.client.get_ptz_position(PtzDescription(name=ptz_name))

    def set_ptz_position(self, ptz_name, pan, tilt, zoom, blocking=False):
        """
        Set the position of the specified ptz

        Args:
            ptz_name: Name of the ptz
            pan: Set the pan to this value in degrees
            tilt: Set the tilt to this value in degrees
            zoom: Set the zoom to this zoom level
            blocking: If true, block for 3 seconds or until the ptz is within 1 degree of the requested pan and tilt values, and
                      0.5 zoom levels of the requested zoom level
        """
        pan, tilt, zoom = self._clamp_request_to_limits(ptz_name, pan, tilt, zoom)
        self.client.set_ptz_position(
            self._get_ptz_description(ptz_name), pan, tilt, zoom
        )
        if blocking:
            start_time = datetime.datetime.now()
            current_position = self.client.get_ptz_position(
                self._get_ptz_description(ptz_name)
            )
            while (
                datetime.datetime.now() - start_time < datetime.timedelta(seconds=3)
                or not math.isclose(current_position.pan, pan, abs_tol=1)
                or not math.isclose(current_position.tilt, tilt, abs_tol=1)
                or not math.isclose(current_position.zoom, zoom, abs_tol=0.5)
            ):
                current_position = self.client.get_ptz_position(
                    self._get_ptz_description(ptz_name)
                )
                time.sleep(0.2)

    def get_ptz_velocity(self, ptz_name) -> PtzVelocity:
        """
        Get the velocity of the ptz with the given name

        Args:
            ptz_name: Name of the ptz

        Returns:
            ptz velocity proto
        """
        return self.client.get_ptz_velocity(PtzDescription(name=ptz_name))

    def set_ptz_velocity(self, ptz_name, pan, tilt, zoom):
        """
        Set the velocity of the various axes of the specified ptz

        Args:
            ptz_name: Name of the ptz
            pan: Set the pan to this value in degrees per second
            tilt: Set the tilt to this value in degrees per second
            zoom: Set the zoom to this value in zoom level per second
        """
        # We do not clamp the velocity to the limits, as it is a rate
        self.client.set_ptz_velocity(
            self._get_ptz_description(ptz_name), pan, tilt, zoom
        )

    def initialise_lens(self):
        """
        Initialises or resets ptz autofocus
        """
        self.client.initialize_lens()


class ImageStreamWrapper:
    """
    A wrapper for the image stream from WebRTC.

    It's not recommended to use images from this image stream for anything beyond casual viewing as they are heavily
    compressed. Prefer use of MediaLog methods to get uncompressed high resolution images.

    Can view the same stream at https://192.168.50.3:31102/h264.sdp.html (depending on the IP of the robot)

    Contains functions adapted from
    https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/spot_cam/webrtc.py
    """

    def __init__(
        self,
        hostname: str,
        robot,
        logger,
        sdp_port=31102,
        sdp_filename="h264.sdp",
        cam_ssl_cert_path=None,
    ):
        """
        Initialise the wrapper

        Args:
            hostname: Hostname/IP of the robot
            robot: Handle for the robot the camera is on
            logger: Logger to use
            sdp_port: SDP port of Spot's WebRTC server
            sdp_filename: File being streamed from the WebRTC server
            cam_ssl_cert_path: Path to the Spot CAM's client cert to check with Spot CAM server
        """
        self.shutdown_flag = threading.Event()
        self.logger = logger
        self.last_image_time = None
        self.image_lock = threading.Lock()
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        config = RTCConfiguration(iceServers=[])
        self.client = WebRTCClient(
            hostname,
            sdp_port,
            sdp_filename,
            cam_ssl_cert_path if cam_ssl_cert_path else False,
            robot.user_token,
            config,
        )

        asyncio.gather(
            self.client.start(),
            self._process_func(),
            self._monitor_shutdown(),
        )
        # Put the async loop into a separate thread so we can continue initialisation
        self.async_thread = threading.Thread(target=loop.run_forever)
        self.async_thread.start()

    async def _monitor_shutdown(self):
        while not self.shutdown_flag.is_set():
            await asyncio.sleep(1.0)

        self.logger.info("Image stream wrapper received shutdown flag")
        await self.client.pc.close()
        asyncio.get_event_loop().stop()

    async def _process_func(self):
        while asyncio.get_event_loop().is_running():
            try:
                frame = await self.client.video_frame_queue.get()

                pil_image = frame.to_image()
                cv_image = np.array(pil_image)
                # OpenCV needs BGR
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                with self.image_lock:
                    self.last_image_time = datetime.datetime.now()
                    self.last_image = cv_image
            except Exception as e:
                self.logger.error(f"Image stream wrapper exception {e}")
            try:
                # discard audio frames
                while not self.client.audio_frame_queue.empty():
                    await self.client.audio_frame_queue.get()
            except Exception as e:
                self.logger.error(
                    f"Image stream wrapper exception while discarding audio frames {e}"
                )

        self.shutdown_flag.set()


class SpotCamWrapper:
    def __init__(self, hostname, username, password, logger):
        self._hostname = hostname
        self._username = username
        self._password = password
        self._logger = logger

        # Create robot object and authenticate.
        self.sdk = bosdyn.client.create_standard_sdk("Spot CAM Client")
        spot_cam.register_all_service_clients(self.sdk)

        self.robot = self.sdk.create_robot(self._hostname)
        SpotWrapper.authenticate(
            self.robot, self._username, self._password, self._logger
        )

        self.payload_client: PayloadClient = self.robot.ensure_client(
            PayloadClient.default_service_name
        )
        self.payload_details = None
        for payload in self.payload_client.list_payloads():
            if payload.is_enabled and "Spot CAM" in payload.name:
                self.payload_details = payload

        if not self.payload_details:
            raise SystemError(
                "Expected an enabled payload with Spot CAM in the name. This does not appear to exist. "
                "Please verify that the spot cam is correctly configured in the payload list on the "
                "admin interface"
            )

        self.lighting = LightingWrapper(self.robot, self._logger)
        self.power = PowerWrapper(self.robot, self._logger)
        self.compositor = CompositorWrapper(self.robot, self._logger)
        self.image = ImageStreamWrapper(self._hostname, self.robot, self._logger)
        self.health = HealthWrapper(self.robot, self._logger)
        self.audio = AudioWrapper(self.robot, self._logger)
        self.stream_quality = StreamQualityWrapper(self.robot, self._logger)
        self.media_log = MediaLogWrapper(self.robot, self._logger)
        self.ptz = PTZWrapper(self.robot, self._logger)

        self._logger.info("Finished setting up spot cam wrapper components")

    def shutdown(self):
        self._logger.info("Shutting down Spot CAM wrapper")
        self.image.shutdown_flag.set()
