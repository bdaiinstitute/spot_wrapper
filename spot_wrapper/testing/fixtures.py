# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import concurrent.futures
import dataclasses
import functools
import inspect
import typing

import grpc
import pytest

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
    api: BaseMockSpot


def fixture(
    cls: typing.Optional[typing.Type[BaseMockSpot]] = None,
    *,
    address: str = "127.0.0.1",
    max_workers: int = 10,
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
        def fixturefunc(monkeypatch, **kwargs) -> typing.Iterator[SpotFixture]:
            with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as thread_pool:
                server = grpc.server(thread_pool)
                port = server.add_insecure_port(f"{address}:0")
                with cls(**kwargs) as mock:
                    mock.add_to(server)
                    server.start()
                    try:
                        with monkeypatch.context() as m:

                            def mock_secure_channel(target, _, *args, **kwargs):
                                return grpc.insecure_channel(target, *args, **kwargs)

                            m.setattr(grpc, "secure_channel", mock_secure_channel)
                            yield SpotFixture(address=address, port=port, api=mock)
                    finally:
                        server.stop(grace=None)

        functools.update_wrapper(fixturefunc, cls)
        sig = inspect.signature(fixturefunc)
        if "monkeypatch" not in sig.parameters:
            sig = sig.replace(
                parameters=(
                    inspect.Parameter("monkeypatch", inspect.Parameter.POSITIONAL_OR_KEYWORD),
                    *sig.parameters.values(),
                )
            )
            fixturefunc.__signature__ = sig  # noqa

        return pytest.fixture(fixturefunc, **kwargs)

    if cls is None:
        return decorator
    return decorator(cls)
