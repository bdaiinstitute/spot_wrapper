from setuptools import setup, find_packages

setup(
    name="spot_wrapper",
    version="1.0.0",
    description="Wrapper for Boston Dynamics Spot SDK",
    packages=["spot_wrapper"],
    install_requires=["bosdyn-client", "bosdyn-api", "bosdyn-mission", "bosdyn-core"],
)
