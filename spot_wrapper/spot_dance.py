import logging
import os
import tempfile
import time
from typing import List, Tuple, Union

from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    ChoreographySequence,
    ChoreographyStatusResponse,
    ExecuteChoreographyResponse,
    MoveParams,
    StartRecordingStateResponse,
    StopRecordingStateResponse,
    UploadAnimatedMoveResponse,
)
from bosdyn.choreography.client.animation_file_to_proto import (
    convert_animation_file_to_proto,
)
from bosdyn.choreography.client.choreography import (
    AnimationValidationFailedError,
    ChoreographyClient,
)
from bosdyn.client import ResponseError
from bosdyn.client.common import FutureWrapper
from bosdyn.client.exceptions import UnauthenticatedError
from bosdyn.client.robot import Robot
from google.protobuf import text_format


class SpotDance:
    def __init__(
        self,
        robot: Robot,
        choreography_client: ChoreographyClient,
        logger: logging.Logger,
    ):
        self._robot = robot
        self._choreography_client = choreography_client
        self._logger = logger

    def upload_animation(self, animation_name: str, animation_file_content: str) -> Tuple[bool, str]:
        """uploads an animation file"""
        # Load the animation file by saving the content to a temp file
        with tempfile.TemporaryDirectory() as temp_dir:
            filename = os.path.join(temp_dir, animation_name + ".cha")
            with open(filename, "w") as tmp:
                tmp.write(animation_file_content)
            try:
                animation_pb = convert_animation_file_to_proto(filename).proto
                return self.upload_animation_proto(animation_pb)
            except Exception as e:
                return (
                    False,
                    f"Failed to convert animation file to protobuf message: {e}",
                )

    def upload_animation_proto(self, animation: Animation) -> Tuple[bool, str]:
        result = False
        result_message = ""
        try:
            self._logger.info(f"Uploading the name {animation.name}")
            upload_response = self._choreography_client.upload_animated_move(animation, animation.name)
            result = upload_response.status == UploadAnimatedMoveResponse.STATUS_OK
            if result:
                result_message = "Successfully uploaded"
                if upload_response.warnings:
                    result_message += f" with warnings from validator {upload_response.warnings}"
            else:
                result_message = (
                    f"Failed to upload animation with status {upload_response.status} and warnings:"
                    f" {upload_response.warnings}"
                )
        except AnimationValidationFailedError as e:
            result_message = f"Failed to upload animation: {e}"
            if e.response.warnings:
                result_message += f" with warnings from validator {e.response.warnings}"
            self._logger.info(f"upload result {result_message}")
            return result, result_message
        except Exception as e:
            result_message = f"Failed to upload animation: {e}"
            return result, result_message
        return result, result_message

    def list_all_dances(self) -> Tuple[bool, str, List[str]]:
        """list all uploaded dances"""
        try:
            dances = self._choreography_client.list_all_sequences().sequence_info
            dances = [dance.name for dance in dances]
            return True, "success", dances
        except Exception as e:
            return (
                False,
                f"request to choreography client for dances failed. Msg: {e}",
                [],
            )

    def list_all_moves(self) -> Tuple[bool, str, List[str]]:
        """list all uploaded moves"""
        try:
            moves = self._choreography_client.list_all_moves().moves
            moves = [move.name for move in moves]
            return True, "success", moves
        except Exception as e:
            return (
                False,
                f"request to choreography client for moves failed. Msg: {e}",
                [],
            )

    def get_choreography_status(self) -> Tuple[bool, str, ChoreographyStatusResponse]:
        """get status of choreography playback"""
        try:
            (status, client_time) = self._choreography_client.get_choreography_status()
            return True, "success", status
        except Exception as e:
            response = ChoreographyStatusResponse()
            response.status = ChoreographyStatusResponse.STATUS_UNKNOWN
            return (
                False,
                f"request to choreography client for status failed. Msg: {e}",
                response,
            )

    def start_recording_state(self, duration_seconds: float) -> Tuple[bool, str, StartRecordingStateResponse]:
        """start recording robot motion as choreography"""
        try:
            status = self._choreography_client.start_recording_state(duration_seconds)
            return True, "success", status
        except Exception as e:
            empty = StartRecordingStateResponse()
            return (
                False,
                f"request to choreography client to start recording failed. Msg: {e}",
                empty,
            )

    def stop_recording_state(self) -> Tuple[bool, str, StopRecordingStateResponse]:
        """stop recording robot motion as choreography"""
        try:
            status = self._choreography_client.stop_recording_state()
            return True, "success", status
        except Exception as e:
            empty = StopRecordingStateResponse()
            return (
                False,
                f"request to choreography client to stop recording failed. Msg: {e}",
                empty,
            )

    def choreography_log_to_animation_file(
        self, name: str, fpath: str, has_arm: bool, **kwargs
    ) -> Tuple[bool, str, str]:
        """save a choreography log to a file as an animation"""
        try:
            file_name = self._choreography_client.choreography_log_to_animation_file(name, fpath, has_arm, **kwargs)
            return True, "success", file_name
        except Exception as e:
            return (
                False,
                f"request to turn log into animation file failed. Msg: {e}",
                "",
            )

    def stop_choreography(self) -> Tuple[bool, str]:
        """
        Issue an empty choreography sequence to stop the robot from dancing
        """
        CHOREO_NAME = "stop_seq"
        empty_move = MoveParams()
        empty_move.type = "ripple_color"
        empty_move.start_slice = 1
        empty_move.requested_slices = 4

        template_sequence = ChoreographySequence()
        template_sequence.name = CHOREO_NAME
        template_sequence.slices_per_minute = 1
        template_sequence.moves.extend([empty_move])

        upload_res, upload_msg = self.upload_choreography(template_sequence)
        # Try ro execute regardless of upload success - may have uploaded successfully in the past
        execute_res, execute_msg = self.execute_choreography_by_name(CHOREO_NAME, start_slice=0)
        combined_message = f"Stop upload msg: {upload_msg}\n, Stop execute message: {execute_msg}"
        return execute_res, combined_message

    def execute_choreography_by_name(
        self, choreography_name: str, start_slice: int, use_async: bool = False
    ) -> Union[FutureWrapper, Tuple[bool, str]]:
        """
        Execute choreography that has already been uploaded to the robot.
        Returns a future wrapper when use_async is true, success, msg tuple otherwise
        """
        client_start_time = time.time()

        try:
            if use_async:
                return self._choreography_client.execute_choreography_async(
                    choreography_name=choreography_name,
                    client_start_time=client_start_time,
                    choreography_starting_slice=start_slice,
                )
            else:
                execute_response = self._choreography_client.execute_choreography(
                    choreography_name=choreography_name,
                    client_start_time=client_start_time,
                    choreography_starting_slice=start_slice,
                )
                result = execute_response.status == ExecuteChoreographyResponse.STATUS_OK
                msg = "Success" if result else "Failure"
                return (result, msg)
        except Exception as e:
            error_msg = f"Exception: {e}"
            return (False, error_msg)

    def upload_choreography(self, choreography_sequence: ChoreographySequence) -> Tuple[bool, str]:
        """Upload choreography sequence for later playback"""
        try:
            self._choreography_client.upload_choreography(choreography_sequence, non_strict_parsing=True)
        except UnauthenticatedError:
            error_msg = (
                "The robot license must contain 'choreography' permissions to upload and execute dances. "
                "Please contact Boston Dynamics Support to get the appropriate license file. "
            )
            return False, error_msg
        except ResponseError as err:
            error_msg = "Choreography sequence upload failed. The following warnings were produced: "
            for warn in err.response.warnings:
                error_msg += warn
            return False, error_msg

        return True, "Success"

    def execute_dance(self, data: Union[ChoreographySequence, str], start_slice: int) -> Tuple[bool, str]:
        """Upload and execute dance. Data can be passed as
        - ChoreographySequence: proto passed directly to function
        - str: file contents of a .csq read directly from disk
        """
        if self._robot.is_estopped():
            error_msg = "Robot is estopped. Please use an external E-Stop client"
            "such as the estop SDK example, to configure E-Stop."
            return False, error_msg

        # Identify the sequence format
        choreography = data
        if isinstance(data, str):
            try:
                choreography = ChoreographySequence()
                text_format.Merge(data, choreography)
            except Exception as execp:
                error_msg = "Failed to read choreography from file. Raised exception: " + str(execp)
                return False, error_msg

        (result, message) = self.upload_choreography(choreography)

        if not result:
            return (result, message)

        try:
            # Setup common response in case of exception
            result_msg = f"Choreography uploaded with message: {message} \n"
            self._robot.power_on()
            (result, message) = self.execute_choreography_by_name(
                choreography.name, start_slice=start_slice, use_async=False
            )

            if result:
                result_msg += "Success: Dance Execution"
            else:
                result_msg = f"Dance Execution failed with message: {message}"

            total_choreography_slices = 0
            for move in choreography.moves:
                total_choreography_slices += move.requested_slices
                estimated_time_seconds = total_choreography_slices / choreography.slices_per_minute * 60.0
            time.sleep(estimated_time_seconds)
            return result, result_msg
        except Exception as e:
            result_msg += f"Error executing dance: {e}"
            return False, result_msg
