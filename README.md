# Spot Wrapper

![Python](https://img.shields.io/badge/python-3.10-blue)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
[![Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![CI](https://github.com/bdaiinstitute/spot_wrapper/workflows/CI/badge.svg)](https://github.com/bdaiinstitute/spot_wrapper/actions)
[![Coverage Status](https://coveralls.io/repos/github/bdaiinstitute/spot_wrapper/badge.svg?branch=main)](https://coveralls.io/github/bdaiinstitute/spot_wrapper?branch=main)
[![LICENSE](https://img.shields.io/badge/license-MIT-purple)](LICENSE)

This Python package is a wrapper around the [Boston Dynamics Spot SDK](https://dev.bostondynamics.com), intended as a united point of entry to the SDK for use with the [spot_ros](https://github.com/heuristicus/spot_ros) and [spot_ros2](https://github.com/bdaiinstitute/spot_ros2) packages.

This package currently corresponds to `spot-sdk` release 5.0.0. The minimum supported version of Python this package supports is 3.10.

# Installation

To install this package clone it and run

```commandline
pip3 install -e .
```

The `-e` flag means that you will not have to reinstall the package when pulling or making changes.

# Updating

To update requirements.txt, use

```commandline
pipreqs . --force
```

# Contributing
This repository enforces `ruff` and `black` linting. To verify that your code will pass inspection, install `pre-commit` and run:
```bash
pre-commit install
pre-commit run --all-files
```
The [Google Style Guide](https://google.github.io/styleguide/) is followed for default formatting. 
