from setuptools import setup, find_packages

setup(
    name="spot_wrapper",
    version="1.0.0",
    description="Wrapper for Boston Dynamics Spot SDK",
    packages=["spot_wrapper"],
    scripts=[
        "tests/test_graph_nav_util.py",
    ],
    install_requires=["bosdyn-client", "bosdyn-api", "bosdyn-mission", "bosdyn-core"],
)
