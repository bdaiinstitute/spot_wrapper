# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import concurrent.futures
import dataclasses
import functools
import pathlib
import typing

import grpc
import pytest

from spot_wrapper.testing.credentials import (
    DEFAULT_TESTING_CERTIFICATES,
    SpotSSLCertificates,
)
from spot_wrapper.testing.mocks import BaseMockSpot


@dataclasses.dataclass
class SpotFixture:
    """
    A fixture that emulates a Spot robot.

    Attributes:
        address: address for the emulated Spot robot gRPC server.
        port: port for the emulated Spot robot gRPC server.
        api: mocked API for the emulated Spot robot.
    """

    address: str
    port: int
    certificate_path: pathlib.Path
    api: BaseMockSpot


def fixture(
    cls: typing.Optional[typing.Type[BaseMockSpot]] = None,
    *,
    address: str = "127.0.0.1",
    max_workers: int = 10,
    certificates: SpotSSLCertificates = DEFAULT_TESTING_CERTIFICATES,
    **kwargs: typing.Any,
) -> typing.Callable:
    """
    A `pytest.fixture` of a mocked Spot robot.

    This function takes a Spot mock class by decorating it. Spot mock classes may
    request parameters like any `pytest.fixture` would do by specifying them in their
    __init__ methods.

    Mocked Spot robots are served through insecure gRPC channels. To enable Spot SDK
    use, this fixture patches gRPC secure channel creation, forcing them to be insecure.

    Args:
        address: address for underlying gRPC server.
        port: port number for underlying gRPC server.
        max_workers: maximum number of threads to use for gRPC call servicing.

    Other keyword arguments are forwarded to `pytest.fixture`.
    """

    def decorator(cls: typing.Type[BaseMockSpot]) -> typing.Callable:
        def fixturefunc(**kwargs) -> typing.Iterator[SpotFixture]:
            with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as thread_pool:
                server = grpc.server(thread_pool)
                robot_certificate_key = certificates.robot_certificate_key_path.read_bytes()
                server_credentials = grpc.ssl_server_credentials(
                    [
                        (robot_certificate_key, certificate_path.read_bytes())
                        for certificate_path in certificates.robot_certificate_paths
                    ]
                )
                port = server.add_secure_port(f"{address}:0", server_credentials)
                with cls(**kwargs) as mock:
                    mock.add_to(server)
                    server.start()
                    try:
                        yield SpotFixture(
                            address=address,
                            port=port,
                            api=mock,
                            certificate_path=certificates.root_certificate_path,
                        )
                    finally:
                        server.stop(grace=None)

        functools.update_wrapper(fixturefunc, cls)
        return pytest.fixture(fixturefunc, **kwargs)

    if cls is None:
        return decorator
    return decorator(cls)
