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
from spot_wrapper.testing.services import BaseSpotServicer


@dataclasses.dataclass
class SpotFixture:
    """
    A fixture that provides Spot robot services.

    Attributes:
        address: address of the underlying gRPC server.
        port: port of the underlying gRPC server.
        certificate_path: SSL certificates used by the underlying gRPC server.
        api: access to Spot services implementation.
    """

    address: str
    port: int
    certificate_path: pathlib.Path
    api: BaseSpotServicer


def fixture(
    cls: typing.Optional[typing.Type[BaseSpotServicer]] = None,
    *,
    address: str = "127.0.0.1",
    max_workers: int = 10,
    certificates: SpotSSLCertificates = DEFAULT_TESTING_CERTIFICATES,
    **kwargs: typing.Any,
) -> typing.Callable:
    """
    Spot robot services as a `pytest.fixture`.

    This function decorates a class that implements Spot robot services.
    Such classes request parameters like any `pytest.fixture` would do by specifying
    them in their __init__ methods.

    Args:
        address: address for the underlying gRPC server.
        port: port number for the underlying gRPC server.
        certificate_path: SSL certificates for the the underlying gRPC server.
        max_workers: maximum number of threads to use for gRPC call servicing.

    Other keyword arguments are forwarded to `pytest.fixture`.
    """

    def decorator(cls: typing.Type[BaseSpotServicer]) -> typing.Callable:
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
