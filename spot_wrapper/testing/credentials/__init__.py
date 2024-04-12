# Copyright (c) 2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import dataclasses
import pathlib
import typing

import pkg_resources


@dataclasses.dataclass
class SpotSSLCertificates:
    root_certificate_path: pathlib.Path
    robot_certificate_key_path: pathlib.Path
    robot_certificate_paths: typing.Sequence[pathlib.Path]


DEFAULT_TESTING_CERTIFICATES = SpotSSLCertificates(
    robot_certificate_key_path=pathlib.Path(
        pkg_resources.resource_filename("spot_wrapper.testing.credentials", "robot.pem")
    ),
    robot_certificate_paths=[
        pathlib.Path(pkg_resources.resource_filename("spot_wrapper.testing.credentials", certificate_filename))
        for certificate_filename in (
            "api.spot.robot.crt",
            "auth.spot.robot.crt",
            "id.spot.robot.crt",
            "payload-registration.spot.robot.crt",
        )
    ],
    root_certificate_path=pathlib.Path(pkg_resources.resource_filename("spot_wrapper.testing.credentials", "ca.crt")),
)
