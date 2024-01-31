# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import inspect
import logging
import os
import queue
import sys
import threading
import typing

import grpc

from spot_wrapper.testing.helpers import GeneralizedDecorator


def implemented(function: typing.Callable) -> bool:
    """
    Checks if a given `function` is implemented or not.

    To do so, its source code is inspected looking for NotImplementedError exception use.
    """
    if inspect.ismethod(function):
        function = function.__func__
    if not hasattr(function, "__code__"):
        raise ValueError(f"no code associated to {function}")
    return "NotImplementedError" not in function.__code__.co_names


class GenericRpcHandlerAccumulator:
    """A helper class to use in place of a `grpc.Server` to inspect handlers."""

    def __init__(self) -> None:
        self.handlers: typing.List[grpc.GenericRpcHandler] = []

    @property
    def service_types(self) -> typing.Iterable[str]:
        """Yields all service types known to the accumulator."""
        for handler in self.handlers:
            if hasattr(handler, "service_name"):
                yield handler.service_name()

    @property
    def method_handlers(
        self,
    ) -> typing.Iterable[typing.Tuple[str, grpc.RpcMethodHandler]]:
        """Yields all RPC method handlers known to the accumulator."""
        for handler in self.handlers:
            if hasattr(handler, "_method_handlers"):
                yield from handler._method_handlers.items()

    def add_generic_rpc_handlers(self, handlers: typing.Iterable[grpc.GenericRpcHandler]) -> None:
        """Implements `grpc.Server.add_generic_rcp_handlers`."""
        self.handlers.extend(handlers)


def collect_servicer_add_functions(
    servicer_class: typing.Any,
) -> typing.Iterable[typing.Callable]:
    """Yields all generated ``add_*_to_server`` functions associated with the given `servicer_class`."""
    for cls in servicer_class.__mro__:
        module = sys.modules[cls.__module__]
        servicer_add_function_name = f"add_{cls.__name__}_to_server"
        servicer_add = getattr(module, servicer_add_function_name, None)
        if callable(servicer_add):
            yield servicer_add


def collect_method_handlers(
    servicer: typing.Any,
) -> typing.Iterable[typing.Tuple[str, grpc.RpcMethodHandler]]:
    """Yields all RPC method handlers for the given `servicer`."""
    accumulator = GenericRpcHandlerAccumulator()
    for add in collect_servicer_add_functions(servicer.__class__):
        add(servicer, accumulator)
    yield from accumulator.method_handlers


def collect_service_types(servicer: typing.Any) -> typing.Iterable[str]:
    """Yields all service type names for the given `servicer`."""
    accumulator = GenericRpcHandlerAccumulator()
    for add in collect_servicer_add_functions(servicer.__class__):
        add(servicer, accumulator)
    yield from accumulator.service_types


class AutoServicer(object):
    """
    A mocking gRPC servicer to ease testing.

    Attributes:
        autospec: if true, deferred handlers will be used in place for every
        non-implemented method handler.
        autotrack: if true, tracking handlers will decorate every method handler.
    """

    autospec = False
    autotrack = False

    def __init__(self, *args: typing.Any, **kwargs: typing.Any) -> None:
        super().__init__(*args, **kwargs)
        self.needs_shutdown: typing.List[typing.Any] = []
        for name, handler in collect_method_handlers(self):
            if handler.response_streaming:
                if handler.request_streaming:
                    unqualified_name = handler.stream_stream.__name__
                    underlying_callable = handler.stream_stream
                    if self.autospec and not implemented(underlying_callable):
                        underlying_callable = DeferredStreamRpcHandler(underlying_callable)
                        self.needs_shutdown.append(underlying_callable)
                    if self.autotrack:
                        underlying_callable = TrackingStreamStreamRpcHandler(underlying_callable)
                    if underlying_callable is not handler.stream_stream:
                        setattr(self, unqualified_name, underlying_callable)
                else:
                    unqualified_name = handler.unary_stream.__name__
                    underlying_callable = handler.unary_stream
                    if self.autospec and not implemented(underlying_callable):
                        underlying_callable = DeferredStreamRpcHandler(underlying_callable)
                        self.needs_shutdown.append(underlying_callable)
                    if self.autotrack:
                        underlying_callable = TrackingUnaryStreamRpcHandler(underlying_callable)
                    if underlying_callable is not handler.unary_stream:
                        setattr(self, unqualified_name, underlying_callable)
            else:
                if handler.request_streaming:
                    unqualified_name = handler.stream_unary.__name__
                    underlying_callable = handler.stream_unary
                    if self.autospec and not implemented(underlying_callable):
                        underlying_callable = DeferredUnaryRpcHandler(underlying_callable)
                        self.needs_shutdown.append(underlying_callable)
                    if self.autotrack:
                        underlying_callable = TrackingStreamUnaryRpcHandler(underlying_callable)
                    if underlying_callable is not handler.stream_unary:
                        setattr(self, unqualified_name, underlying_callable)
                else:
                    unqualified_name = handler.unary_unary.__name__
                    underlying_callable = handler.unary_unary
                    if self.autospec and not implemented(underlying_callable):
                        underlying_callable = DeferredUnaryRpcHandler(underlying_callable)
                        self.needs_shutdown.append(underlying_callable)
                    if self.autotrack:
                        underlying_callable = TrackingUnaryUnaryRpcHandler(underlying_callable)
                    if underlying_callable is not handler.unary_unary:
                        setattr(self, unqualified_name, underlying_callable)

    def add_to(self, server: grpc.Server) -> None:
        """Adds all service handlers to server."""
        for add in collect_servicer_add_functions(self.__class__):
            add(self, server)

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.shutdown()

    def shutdown(self):
        """
        Shutdown what needs to be shutdown.

        Typically, deferred RPC handlers.
        """
        for obj in self.needs_shutdown:
            obj.shutdown()


class TrackingUnaryUnaryRpcHandler(GeneralizedDecorator):
    """A decorator for unary-unary gRPC handlers that tracks calls."""

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self.requests: typing.List = []
        self.num_calls = 0

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        try:
            self.requests.append(request)
            return self.__wrapped__(request, context)
        finally:
            self.num_calls += 1


class TrackingStreamUnaryRpcHandler(GeneralizedDecorator):
    """A decorator for stream-unary gRPC handlers that tracks calls."""

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self.requests: typing.List = []
        self.num_calls = 0

    def __call__(self, request_iterator: typing.Iterator, context: grpc.ServicerContext) -> typing.Any:
        try:
            request = list(request_iterator)
            self.requests.append(request)
            request_iterator = iter(request)
            return self.__wrapped__(request_iterator, context)
        finally:
            self.num_calls += 1


class TrackingUnaryStreamRpcHandler(GeneralizedDecorator):
    """A decorator for unary-stream gRPC handlers that tracks calls."""

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self.requests: typing.List = []
        self.num_calls = 0

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Iterator:
        try:
            self.requests.append(request)
            yield from self.__wrapped__(request, context)
        finally:
            self.num_calls += 1


class TrackingStreamStreamRpcHandler(GeneralizedDecorator):
    """A decorator for stream-stream gRPC handlers that tracks calls."""

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self.requests: typing.List = []
        self.num_calls = 0

    def __call__(self, request_iterator: typing.Iterator, context: grpc.ServicerContext) -> typing.Iterator:
        try:
            request = list(request_iterator)
            self.requests.append(request)
            request_iterator = iter(request)
            yield from self.__wrapped__(request_iterator, context)
        finally:
            self.num_calls += 1


class DeferredRpcHandler(GeneralizedDecorator):
    """
    A gRPC handler that decouples invocation and computation execution paths.

    By default, a call or invocation blocks the calling thread waiting for resolution,
    which is thus deferred to another thread. Both request and context are made available
    to the thread resolving the call or invocation. The response is fed back to the original
    thread, which is then unblocked.

    Invocations may also be resolved in advance, specifying future responses in absence of a request.
    """

    class Call:
        """A deferred gRPC call to be resolved."""

        def __init__(self, request: typing.Any, context: grpc.ServicerContext) -> None:
            """
            Args:
                request: gRPC request object.
                context: gRPC servicing context.
            """
            self._request = request
            self._context = context
            self._response: typing.Optional[typing.Any] = None
            self._code: typing.Optional[grpc.StatusCode] = None
            self._details: typing.Optional[str] = None
            self._completed = False
            self._completion = threading.Condition()

        @property
        def request(self) -> typing.Any:
            """Call request."""
            return self._request

        @property
        def context(self) -> grpc.ServicerContext:
            """Call context."""
            return self._context

        @property
        def response(self) -> typing.Optional[typing.Any]:
            """Call response, if complete and successful."""
            return self._response

        @property
        def code(self) -> typing.Optional[grpc.StatusCode]:
            """Call code, if complete and failed."""
            return self._code

        @property
        def details(self) -> typing.Optional[str]:
            """Call details, if complete and failed."""
            return self._details

        def wait_for_completion(self, timeout: typing.Optional[float] = None) -> bool:
            """
            Waits for call completion.

            Args:
                timeout: time in seconds to wait for call completion.
                If none is provided, it will wait indefinitely.

            Returns:
                true if call is completed, false otherwise.
            """
            with self._completion:
                return self._completed or self._completion.wait(timeout)

        def returns(self, response: typing.Any) -> None:
            """Succeeds the call by returning a `response`."""
            with self._completion:
                if self._completed:
                    raise RuntimeError("call already completed!")
                self._response = response
                self._completed = True
                self._completion.notify_all()

        def fails(self, code: grpc.StatusCode, details: typing.Optional[str] = None) -> None:
            """Fails the call by setting an error `code` and optional `details`."""
            with self._completion:
                if self._completed:
                    raise RuntimeError("call already completed!")
                self._code = code
                self._details = details
                self._completed = True
                self._completion.notify_all()

    class Future:
        """
        A window to future gRPC calls.

        This helper class stores call resolutions in a FIFO queue,
        for the handler to apply on future calls.
        """

        def __init__(self) -> None:
            self._changequeue: queue.SimpleQueue = queue.SimpleQueue()

        def materialize(self, call: "DeferredRpcHandler.Call") -> bool:
            """Makes `call` the next call, applying the oldest resolution specified."""
            try:
                change = self._changequeue.get_nowait()
                change(call)
                return True
            except queue.Empty:
                return False

        def returns(self, response: typing.Any) -> None:
            """Specifies the next call will succeed with the given `response`."""
            self._changequeue.put(lambda call: call.returns(response))

        def fails(self, code: grpc.StatusCode, details: typing.Optional[str] = None) -> None:
            """Specifies the next call will fail with given error `code` and `details`."""
            self._changequeue.put(lambda call: call.fails(code, details))

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self._future = DeferredRpcHandler.Future()
        self._callqueue: queue.SimpleQueue = queue.SimpleQueue()

    def shutdown(self) -> None:
        """Shutdown handler, aborting all pending calls."""
        while not self._callqueue.empty():
            call = self._callqueue.get_nowait()
            if "PYTEST_CURRENT_TEST" in os.environ:
                logging.warning(f"{self.__name__} call not served, dropped during shutdown")
            call.fails(grpc.StatusCode.ABORTED, "call dropped")

    @property
    def pending(self) -> bool:
        """Whether a call is waiting to be served."""
        return not self._callqueue.empty()

    def serve(self, timeout: typing.Optional[float] = None) -> typing.Optional["DeferredRpcHandler.Call"]:
        """
        Serve the next pending call, if any.

        Args:
            timeout: time in seconds to wait for a call.
            If none is provided, it will wait indefinitely.

        Returns:
            a deferred call to be served or none on timeout.
        """
        try:
            return self._callqueue.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def future(self) -> "DeferredRpcHandler.Future":
        """The window to future calls."""
        return self._future


class DeferredUnaryRpcHandler(DeferredRpcHandler):
    """A gRPC any-unary handler that decouples invocation and computation execution paths."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        call = DeferredRpcHandler.Call(request, context)
        if not self._future.materialize(call):
            self._callqueue.put(call)
            call.wait_for_completion()
        if not call.response:
            context.abort(call.code, call.details)
        return call.response


class DeferredStreamRpcHandler(DeferredRpcHandler):
    """A gRPC any-stream handler that decouples invocation and computation execution paths."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        call = DeferredRpcHandler.Call(request, context)
        if not self._future.materialize(call):
            self._callqueue.put(call)
            call.wait_for_completion()
        if call.response:
            yield from call.response
        else:
            context.abort(call.code, call.details)
