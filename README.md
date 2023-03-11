# Spot Wrapper

This Python package is a wrapper around the Boston Dynamics Spot SDK, intended as a united point of entry to the SDK for use with the spot_ros and spot_ros2 packages.

# Installation

To install this package clone it and run

```commandline
pip3 install -e .
```

The `-e` flag means that you will not have to reinstall the package when there are updates.

# Updating

To update requirements.txt, use

```commandline
pipreqs . --force
```