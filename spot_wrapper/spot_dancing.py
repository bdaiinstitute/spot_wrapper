import sys
import time

import bosdyn.client
import bosdyn.client.util
from bosdyn.choreography.client.choreography import (ChoreographyClient,
                                                     load_choreography_sequence_from_txt_file)
from bosdyn.client import ResponseError
from bosdyn.client.exceptions import UnauthenticatedError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.license import LicenseClient



class SpotDancing():
    def __init__(self, hostname: str, robot_name: str):
        self._hostname = hostname
        self._robot_name = robot_name
        sdk = bosdyn.client.create_standard_sdk('UploadChoreography')
        sdk.register_service_client(ChoreographyClient)
        self._robot = sdk.create_robot(self._hostname)
        bosdyn.client.util.authenticate(self._robot)
        license_client = self._robot.ensure_client(LicenseClient.default_service_name)
        if not license_client.get_feature_enabled([ChoreographyClient.license_name
                                              ])[ChoreographyClient.license_name]:
            print("This robot is not licensed for choreography.")
            sys.exit(1)
        lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.acquire()
        lk = LeaseKeepAlive(lease_client)
        self._choreography_client = self._robot.ensure_client(ChoreographyClient.default_service_name)

    def execute_dance(self, filepath: str) -> tuple[bool, list]:
            """uploads and executes the dance at filepath to Spot"""
            if self._robot.is_estopped():
                error_msg = "Robot is estopped. Please use an external E-Stop client"
                "such as the estop SDK example, to configure E-Stop."
                return False, error_msg
            try:
                choreography = load_choreography_sequence_from_txt_file(filepath)
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
                start_slice = 0 #start the choreography at the beginning

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
