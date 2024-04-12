from setuptools import find_packages, setup

setup(
    name="spot_wrapper",
    version="1.0.0",
    description="Wrapper for Boston Dynamics Spot SDK",
    packages=find_packages(include=["spot_wrapper*"]),
    package_data={"spot_wrapper.testing.credentials": ["*.crt", "*.pem"]},
    install_requires=["bosdyn-client", "bosdyn-api", "bosdyn-mission", "bosdyn-core", "grpcio", "inflection", "pytest"],
)
