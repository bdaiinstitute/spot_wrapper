import time
import tempfile
import bosdyn.choreography.client.choreography as bosdyn_choreo

from bosdyn.choreography.client.choreography import (
    load_choreography_sequence_from_txt_file,
)
from bosdyn.client import ResponseError
from bosdyn.client.exceptions import UnauthenticatedError
from bosdyn.client.robot import Robot
from bosdyn.choreography.client.choreography import ChoreographyClient
from bosdyn.choreography.client.animation_file_to_proto import convert_animation_file_to_proto
from bosdyn.api.spot import choreography_sequence_pb2
from google.protobuf import text_format


class SpotDance:
    def __init__(
        self,
        robot: Robot,
        choreography_client: bosdyn_choreo.ChoreographyClient,
        is_licensed_for_choreography: bool,
    ):
        self._robot = robot
        self._choreography_client = choreography_client
        self._is_licensed_for_choreography = is_licensed_for_choreography

    def upload_animation(self, animation_file_content : str) -> tuple[bool, str]:
        """ uploads an animation file """
        # Load the animation file by saving the content to a temp file
        tmp = tempfile.NamedTemporaryFile('wb')
        with open(tmp.name, 'w') as f:
            f.write(animation_file_content)
        animation_pb = convert_animation_file_to_proto(tmp).proto
        try:
            upload_response = self._choreography_client.upload_animated_move(animation_pb)
        except Exception as e:
            error_msg = "Failed to upload animation: {}".format(e)
            return False, error_msg
        return True, "Success"

    def execute_dance(self, data: str) -> tuple[bool, str]:
        """uploads and executes the dance at filepath to Spot"""

        if not self._is_licensed_for_choreography:
            return False, "Robot is not licensed for choreography."
        if self._robot.is_estopped():
            error_msg = "Robot is estopped. Please use an external E-Stop client"
            "such as the estop SDK example, to configure E-Stop."
            return False, error_msg
        try:
            choreography = choreography_sequence_pb2.ChoreographySequence()
            text_format.Merge(data, choreography)
        except Exception as execp:
            error_msg = "Failed to load choreography. Raised exception: " + str(execp)
            return False, error_msg
        try:
            upload_response = self._choreography_client.upload_choreography(
                choreography, non_strict_parsing=True
            )
        except UnauthenticatedError as err:
            error_msg = "The robot license must contain 'choreography' permissions to upload and execute dances."
            "Please contact Boston Dynamics Support to get the appropriate license file. "
            return False, error_msg
        except ResponseError as err:
            error_msg = "Choreography sequence upload failed. The following warnings were produced: "
            for warn in err.response.warnings:
                error_msg += warn
            return False, error_msg

        # Routine is valid! Power on robot and execute routine.
        try:
            self._robot.power_on()
            routine_name = choreography.name
            client_start_time = time.time() + 5.0
            start_slice = 0  # start the choreography at the beginning

            # Issue the command to the robot's choreography service.

            self._choreography_client.execute_choreography(
                choreography_name=routine_name,
                client_start_time=client_start_time,
                choreography_starting_slice=start_slice,
            )

            # Estimate how long the choreographed sequence will take, and sleep that long.

            total_choreography_slices = 0
            for move in choreography.moves:
                total_choreography_slices += move.requested_slices
            estimated_time_seconds = (
                total_choreography_slices / choreography.slices_per_minute * 60.0
            )
            time.sleep(estimated_time_seconds)

            self._robot.power_off()
            return True, "sucess"
        except Exception as e:
            return False, f"Error executing dance: {e}"
