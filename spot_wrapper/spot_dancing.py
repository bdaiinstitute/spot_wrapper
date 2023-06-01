import sys
import time
import logging

import bosdyn.client
import bosdyn.client.util
from bosdyn.choreography.client.choreography import (ChoreographyClient,
                                                     load_choreography_sequence_from_txt_file)
from bosdyn.client import ResponseError
from bosdyn.client.exceptions import UnauthenticatedError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.license import LicenseClient



class SpotDancing():
    def __init__(self, hostname: str, robot_name: str, username: str, password: str, logger:  logging.Logger):
        self._hostname = hostname
        self._robot_name = robot_name
        self._username = username
        self._password = password
        self._logger = logger

        sdk = bosdyn.client.create_standard_sdk('UploadChoreography')
        sdk.register_service_client(ChoreographyClient)
        self._robot = sdk.create_robot(self._hostname)
        if not self._robot:
            print("create robot failed!")
        authenticated = self.authenticate(
            self._robot, self._username, self._password, self._logger
        )
        if not authenticated:
            print("Authentication failed!")
            return
        license_client = self._robot.ensure_client(LicenseClient.default_service_name)
        if not license_client.get_feature_enabled([ChoreographyClient.license_name
                                              ])[ChoreographyClient.license_name]:
            print("This robot is not licensed for choreography.")
            sys.exit(1)
        lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.acquire()
        lk = LeaseKeepAlive(lease_client)
        self._choreography_client = self._robot.ensure_client(ChoreographyClient.default_service_name)

    @staticmethod
    def authenticate(robot, username, password, logger):
        """
        Authenticate with a robot through the bosdyn API. A blocking function which will wait until authenticated (if
        the robot is still booting) or login fails

        Args:
            robot: Robot object which we are authenticating with
            username: Username to authenticate with
            password: Password for the given username
            logger: Logger with which to print messages

        Returns:

        """
        authenticated = False
        while not authenticated:
            try:
                logger.info("Trying to authenticate with robot...")
                robot.authenticate(username, password)
                robot.time_sync.wait_for_sync(10)
                logger.info("Successfully authenticated.")
                authenticated = True
            except RpcError as err:
                sleep_secs = 15
                logger.warn(
                    "Failed to communicate with robot: {}\nEnsure the robot is powered on and you can "
                    "ping {}. Robot may still be booting. Will retry in {} seconds".format(
                        err, robot.address, sleep_secs
                    )
                )
                time.sleep(sleep_secs)
            except bosdyn.client.auth.InvalidLoginError as err:
                logger.error("Failed to log in to robot: {}".format(err))
                raise err

        return authenticated


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
