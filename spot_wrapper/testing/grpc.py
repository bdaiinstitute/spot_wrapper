# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import collections.abc
import concurrent.futures
import inspect
import logging
import math
import os
import queue
import sys
import threading
import typing
import weakref
from abc import ABC, abstractmethod

import grpc
from bosdyn.api.header_pb2 import CommonError

from spot_wrapper.testing.helpers import ForwardingWrapper, cache1


def implemented(function: typing.Callable) -> bool:
    """
    Checks if a given `function` is implemented or not.

    To do so, its source code is inspected looking for NotImplementedError exception use.
    """
    if not callable(function):
        raise ValueError(f"{function} is not a callable")
    if inspect.ismethod(function):
        function = function.__func__
    if not inspect.isfunction(function):
        return True  # generic callable, assume implemented
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


class BaseServicer:
    def add_to(self, server: grpc.Server) -> None:
        """Adds all service handlers to server."""
        for add in collect_servicer_add_functions(self.__class__):
            add(self, server)


class AutoServicer(BaseServicer):
    """
    A mocking gRPC servicer to ease testing.

    Attributes:
        autospec: if true, deferred handlers will be used in place for every
        non-implemented method handler.
        autotrack: if true, tracking handlers will decorate every method handler.
        autocomplete: if true, autocompleting handlers will decorate every method handler.
    """

    autospec = False
    autotrack = False
    autocomplete = False

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
                    if self.autocomplete:
                        underlying_callable = AutoCompletingStreamStreamRpcHandler(underlying_callable)
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
                    if self.autocomplete:
                        underlying_callable = AutoCompletingUnaryStreamRpcHandler(underlying_callable)
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
                    if self.autocomplete:
                        underlying_callable = AutoCompletingStreamUnaryRpcHandler(underlying_callable)
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
                    if self.autocomplete:
                        underlying_callable = AutoCompletingUnaryUnaryRpcHandler(underlying_callable)
                    if underlying_callable is not handler.unary_unary:
                        setattr(self, unqualified_name, underlying_callable)

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


class TrackingUnaryUnaryRpcHandler(ForwardingWrapper):
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


class TrackingStreamUnaryRpcHandler(ForwardingWrapper):
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


class TrackingUnaryStreamRpcHandler(ForwardingWrapper):
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


class TrackingStreamStreamRpcHandler(ForwardingWrapper):
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


def fill_response_header(request: typing.Any, response: typing.Any) -> bool:
    """Fill response header if any when missing."""
    if not hasattr(response, "header"):
        return False
    if hasattr(request, "header"):
        response.header.request_header.CopyFrom(request.header)
        response.header.request_received_timestamp.CopyFrom(request.header.request_timestamp)
    response.header.error.code = response.header.error.code or CommonError.CODE_OK
    return True


class AutoCompletingUnaryUnaryRpcHandler(ForwardingWrapper):
    """A decorator for unary-unary gRPC handlers that autocompletes response headers."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        response = self.__wrapped__(request, context)
        fill_response_header(request, response)
        return response


class AutoCompletingStreamUnaryRpcHandler(ForwardingWrapper):
    """A decorator for stream-unary gRPC handlers that autocompletes response headers.

    The last request chunk header will be used to complete the response header.
    """

    def __call__(self, request_iterator: typing.Iterator, context: grpc.ServicerContext) -> typing.Any:
        cached_request_iterator = cache1(request_iterator)
        response = self.__wrapped__(cached_request_iterator, context)
        fill_response_header(cached_request_iterator.cache, response)
        return response


class AutoCompletingUnaryStreamRpcHandler(ForwardingWrapper):
    """A decorator for unary-stream gRPC handlers that autocompletes response headers."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Iterator:
        for response in self.__wrapped__(request, context):
            fill_response_header(request, response)
            yield response


class AutoCompletingStreamStreamRpcHandler(ForwardingWrapper):
    """A decorator for stream-stream gRPC handlers that autocompletes response headers.

    The last request chunk header will be used to complete the response header.
    """

    def __call__(self, request_iterator: typing.Iterator, context: grpc.ServicerContext) -> typing.Iterator:
        cached_request_iterator = cache1(request_iterator)
        for response in self.__wrapped__(cached_request_iterator, context):
            fill_response_header(cached_request_iterator.cache, response)
            yield response


class DeferredRpcHandler(ForwardingWrapper):
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

        @property
        def completed(self) -> bool:
            """Whether call is complete or not."""
            return self._completed

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

        This helper class stores call outcomes in a FIFO queue,
        for the handler to resolve future calls.
        """

        class Outcome(ABC):
            """A generic outcome for future calls."""

            def __init__(self) -> None:
                self._num_repeats: typing.Optional[typing.Union[int, float]] = None
                self._num_uses: int = 0

            @property
            def num_repeats(self) -> typing.Optional[typing.Union[int, float]]:
                """Returns the number of times this outcome will be used."""
                return self._num_repeats

            @property
            def num_uses(self) -> int:
                """Returns the number of times this outcome has been used."""
                return self._num_uses

            @abstractmethod
            def do_resolve(self, call: "DeferredRpcHandler.Call") -> None:
                pass

            def resolve(self, call: "DeferredRpcHandler.Call") -> bool:
                """Resolves the given `call` using this outcome."""
                num_repeats = float(self._num_repeats or 1)
                if self._num_uses >= num_repeats:
                    return False
                self.do_resolve(call)
                self._num_uses += 1
                return True

            def repeatedly(self, times: int) -> None:
                """States that this outcome can be used repeatedly, a finite number of `times`."""
                assert times > 0
                if self._num_repeats is not None:
                    raise RuntimeError("Outcome repetition already specified")
                self._num_repeats = times

            def forever(self) -> None:
                """States that this outcome can be used repeatedly, forever."""
                if self._num_repeats is not None:
                    raise RuntimeError("Outcome repetition already specified")
                self._num_repeats = math.inf

        class Response(Outcome):
            """A successful response outcome for future calls."""

            def __init__(self, response: typing.Any) -> None:
                super().__init__()
                self._response = response

            def do_resolve(self, call: "DeferredRpcHandler.Call") -> None:
                call.returns(self._response)

            def _maybe_raise_for_streamed_responses(self):
                if isinstance(self._response, collections.abc.Iterable):
                    if not isinstance(self._response, collections.abc.Sized):
                        raise RuntimeError("Cannot repeat an streamed response, specify it as a sized collection")

            def repeatedly(self, times: int) -> None:
                self._maybe_raise_for_streamed_responses()
                super().repeatedly(times)

            def forever(self) -> None:
                self._maybe_raise_for_streamed_responses()
                super().forever()

        class Failure(Outcome):
            """A failure outcome for future calls."""

            def __init__(self, code: grpc.StatusCode, details: typing.Optional[str] = None) -> None:
                super().__init__()
                self._code = code
                self._details = details

            def do_resolve(self, call: "DeferredRpcHandler.Call") -> None:
                call.fails(self._code, self._details)

        def __init__(self) -> None:
            self._lock: threading.Lock = threading.Lock()
            self._queue: collections.deque = collections.deque()

        def materialize(self, call: "DeferredRpcHandler.Call") -> bool:
            """Makes `call` the next call, applying the oldest resolution specified."""
            with self._lock:
                while len(self._queue) > 0:
                    outcome = self._queue[0]
                    if outcome.resolve(call):
                        return True
                    self._queue.popleft()
                return False

        def _raise_when_future_is_predetermined(self) -> None:
            if len(self._queue) > 0:
                pending_outcome = self._queue[-1]
                if pending_outcome.num_repeats and not math.isfinite(pending_outcome.num_repeats):
                    raise RuntimeError("Future is predetermined, cannot specify response (did you use forever())")

        def returns(self, response: typing.Any) -> "DeferredRpcHandler.Future.Outcome":
            """
            Specifies the next call will succeed with the given `response`.

            It returns a future outcome that can inspected and repeated as need be.
            """
            with self._lock:
                self._raise_when_future_is_predetermined()
                outcome = DeferredRpcHandler.Future.Response(response)
                self._queue.append(outcome)
                return outcome

        def fails(
            self, code: grpc.StatusCode, details: typing.Optional[str] = None
        ) -> "DeferredRpcHandler.Future.Outcome":
            """
            Specifies the next call will fail with given error `code` and `details`.

            It returns a future outcome that can inspected and repeated as need be.
            """
            with self._lock:
                self._raise_when_future_is_predetermined()
                outcome = DeferredRpcHandler.Future.Failure(code, details)
                self._queue.append(outcome)
                return outcome

    def __init__(self, handler: typing.Callable) -> None:
        super().__init__(handler)
        self._future = DeferredRpcHandler.Future()
        self._callqueue: queue.SimpleQueue = queue.SimpleQueue()
        self._callset: weakref.WeakSet = weakref.WeakSet()

    def shutdown(self) -> None:
        """Shutdown handler, aborting all pending calls."""
        for call in self._callset:
            if not call.completed:
                if "PYTEST_CURRENT_TEST" in os.environ:
                    logging.warning(f"{self.__name__} call not completed, aborted during shutdown")
                call.fails(grpc.StatusCode.ABORTED, "call aborted")
        self._callset.clear()
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
            call = self._callqueue.get(timeout=timeout)
            self._callset.add(call)
            return call
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


class BackServicer(typing.Protocol):
    """Protocol for a servicer that can handle back requests, typically from a proxy."""

    def submit(self, request: typing.Any) -> concurrent.futures.Future:
        """Submit a unary or streaming request.

        Returns a future unary or streaming response.
        """


class ReverseProxyServicer(BaseServicer):
    """
    A gRPC servicer reverse proxy to relay requests.

    Attributes:
        proxy_services: optional set of services to reverse proxy.
        If none is provided, proxy will apply to all found services.
    """

    proxy_services: typing.Optional[typing.Set[str]] = None

    def __init__(self, *args, server: BackServicer, **kwargs) -> None:
        """Initializes the reverse proxy, binding to the given `server`."""
        super().__init__(*args, **kwargs)
        for endpoint_name, handler in collect_method_handlers(self):
            service_name, _, method_name = endpoint_name.rpartition("/")
            if self.proxy_services is not None:
                if not any(service_name.endswith(proxy_service_name) for proxy_service_name in self.proxy_services):
                    continue
            proxy_cls: typing.Type
            if handler.response_streaming:
                if handler.request_streaming:
                    underlying_callable = handler.stream_stream
                else:
                    underlying_callable = handler.unary_stream
                proxy_cls = ProxiedStreamRpcHandler
            else:
                if handler.request_streaming:
                    underlying_callable = handler.stream_unary
                else:
                    underlying_callable = handler.unary_unary
                proxy_cls = ProxiedUnaryRpcHandler
            if implemented(underlying_callable):
                raise RuntimeError(f"{endpoint_name} is already implemented")
            setattr(self, method_name, proxy_cls(server, underlying_callable))


class ProxiedRpcHandler(ABC):
    """A generic gRPC handler that proxies requests."""

    def __init__(self, server: BackServicer, stub: typing.Callable) -> None:
        for name in ("__module__", "__name__", "__qualname__", "__doc__", "__annotations__", "__type_params__"):
            try:
                value = getattr(stub, name)
            except AttributeError:
                pass
            else:
                setattr(self, name, value)
        self._server = server

    @abstractmethod
    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        ...


class ProxiedUnaryRpcHandler(ProxiedRpcHandler):
    """A gRPC any-unary handler that proxies requests."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Any:
        future = self._server.submit(request)
        return future.result(timeout=context.time_remaining())


class ProxiedStreamRpcHandler(ProxiedRpcHandler):
    """A gRPC any-stream handler that proxies requests."""

    def __call__(self, request: typing.Any, context: grpc.ServicerContext) -> typing.Iterator:
        future = self._server.submit(request)
        yield from future.result(timeout=context.time_remaining())
