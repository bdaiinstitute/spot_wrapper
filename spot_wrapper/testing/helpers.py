# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import functools
import typing

import grpc
from bosdyn.api.header_pb2 import CommonError
from bosdyn.api.lease_pb2 import ResourceTree


class GeneralizedDecorator:
    __name__ = __qualname__ = __doc__ = ""

    __annotations__ = {}

    @staticmethod
    def wraps(wrapped: typing.Callable):
        def decorator(func: typing.Callable):
            class wrapper(GeneralizedDecorator):
                def __call__(
                    self, *args: typing.Any, **kwargs: typing.Any
                ) -> typing.Any:
                    return func(*args, **kwargs)

            return wrapper(wrapped)

        return decorator

    def __init__(self, wrapped: typing.Callable) -> None:
        functools.update_wrapper(self, wrapped, updated=[])

    def __getattr__(self, name: str) -> typing.Any:
        return getattr(self.__wrapped__, name)

    def __call__(self, *args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        raise NotImplementedError()


UnaryUnaryHandlerCallable = typing.Callable[
    [typing.Any, grpc.ServicerContext], typing.Any
]


def enforce_matching_headers(
    handler: UnaryUnaryHandlerCallable,
) -> UnaryUnaryHandlerCallable:
    """Enforce headers for handler request and response match (by copy)."""

    @GeneralizedDecorator.wraps(handler)
    def wrapper(request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        response = handler(request, context)
        if hasattr(request, "header") and hasattr(response, "header"):
            response.header.request_header.CopyFrom(request.header)
            response.header.request_received_timestamp.CopyFrom(
                request.header.request_timestamp
            )
            response.header.error.code = (
                response.header.error.code or CommonError.CODE_OK
            )
        return response

    return wrapper


def walk_resource_tree(resource_tree: ResourceTree) -> typing.Iterable[ResourceTree]:
    """Walks `resource_tree` top-down, depth-first."""
    yield resource_tree
    for subtree in resource_tree.sub_resources:
        yield from walk_resource_tree(subtree)
