# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import functools
import typing

from bosdyn.api.lease_pb2 import ResourceTree


class ForwardingWrapper:
    """A `functools.wraps` equivalent that is transparent to attribute access."""

    __name__ = __qualname__ = __doc__ = ""

    __annotations__ = {}

    @staticmethod
    def wraps(wrapped: typing.Callable):
        def decorator(func: typing.Callable):
            class wrapper(ForwardingWrapper):
                def __call__(self, *args: typing.Any, **kwargs: typing.Any) -> typing.Any:
                    return func(*args, **kwargs)

            return wrapper(wrapped)

        return decorator

    def __init__(self, wrapped: typing.Callable) -> None:
        functools.update_wrapper(self, wrapped, updated=[])

    def __getattr__(self, name: str) -> typing.Any:
        return getattr(self.__wrapped__, name)

    def __call__(self, *args: typing.Any, **kwargs: typing.Any) -> typing.Any:
        raise NotImplementedError()


def walk_resource_tree(resource_tree: ResourceTree) -> typing.Iterable[ResourceTree]:
    """Walks `resource_tree` top-down, depth-first."""
    yield resource_tree
    for subtree in resource_tree.sub_resources:
        yield from walk_resource_tree(subtree)
