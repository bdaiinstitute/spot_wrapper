# Copyright (c) 2023 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

"""
This subpackage provides `pytest` compatible machinery to test Spot SDK use.

At its core, this machinery is nothing but mocks and fixtures: mocks of Spot gRPC
services, and fixtures serving those mocks over the wire.

Mocks are basically gRPC servicer subclasses. Several mock classes, targeting the
many services that Spot robots expose and designed for aggregation via multiple
inheritance, are available under `spot_wrapper.testing.mocks`. In combination,
these classes afford centralized, cohesive mock implementations.

.. note::

   To mock the Spot SDK at the gRPC interface level adds complexity to mock
   implementations, low-level as this interface is. In exchange, these mocks
   have complete control over a well-defined interface that spans the entire
   feature set.

Fixtures are `pytest.fixture`s built around mock classes. Decorating a mock class with
the `spot_wrapper.testing.fixture` function turns it into a fixture. When requested by
a test, a fixture will start a gRPC server to host mocked services for as long as
it remains in scope. This gRPC server will listen at a unique address, which is made
available to the test.

.. code-block:: python

   from spot_wrapper.wrapper import SpotWrapper

   import spot_wrapper.testing
   from spot_wrapper.testing.mocks import MockSpot

   @spot_wrapper.testing.fixture
   class fake_spot(MockSpot):
       pass

   def test_wrapper(fake_spot):
       wrapper = SpotWrapper(
           username="spot",
           password="spot",
           hostname=fake_spot.address,
           port=fake_spot.port,
           robot_name=simple_spot.api.name,
           logger=logging.getLogger("spot"),
       )
       assert wrapper.valid


No built-in fixtures are provided, as these are always user-defined.
`spot_wrapper.testing.mocks.MockSpot` implements enough Spot gRPC
services for both Spot SDK and `SpotWrapper` to initialize, but enough
does not mean all. A mock need not implement all Spot gRPC services, as
long as either non-implemented gRPC services are never called or the mock
is autospec'd. Autospec'd `spot_wrapper.testing.mocks.BaseMockSpot`
subclasses, and `spot_wrapper.testing.mocks.MockSpot` is one such class,
will define _deferred method handlers_ for non-implemented services.
A deferred method handler is a callable that allows resolving gRPC
service calls in advance, specifying their future outcome, and serving
them as they come in the main thread. In a way, these handlers are the
`unittest.mock.Mock` equivalents for gRPC services.

.. code-block:: python

   from spot_wrapper.wrapper import SpotWrapper

   import spot_wrapper.testing
   from spot_wrapper.testing.mocks import MockSpot

   from bosdyn.api.robot_command_pb2 import RobotCommandResponse

   @spot_wrapper.testing.fixture
   class fake_spot(MockSpot):
       pass

   def test_wrapper(fake_spot):
       wrapper = SpotWrapper(
           username="spot",
           password="spot",
           hostname=fake_spot.address,
           port=fake_spot.port,
           robot_name=simple_spot.api.name,
           logger=logging.getLogger("spot"),
       )
       assert wrapper.valid

       response = RobotCommandResponse()
       response.status = RobotCommandResponse.Status.STATUS_OK
       fake_spot.api.RobotCommand.future.returns(response)
       ok, message = wrapper.sit()
       assert ok, message
"""

from spot_wrapper.testing.fixtures import fixture

__all__ = ["fixture"]
