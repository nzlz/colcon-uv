# colcon-uv

[![CI](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml/badge.svg)](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml)

A **colcon extension** for building and testing Python packages that use **[uv](https://github.com/astral-sh/uv)** for dependency management.

Intended, but not limited, to use with **ROS 2**. This extension provides a modern alternative to traditional setuptools-based or even poetry-based Python package management in ROS environments.

## Features

- **Fast Dependency Management**: Leverages UV's lightning-fast dependency resolution and installation
- **Modern Python Packaging**: Support for `pyproject.toml`-based packages following PEP 517/518 standards
- **ROS Integration**: Seamless integration with colcon build system and ROS package management
- **Dependency Isolation**: Prevents dependency conflicts between packages

## Installation

**Important**: This extension must be installed in the same environment as colcon to be discoverable by colcon's plugin system. Typically this means system-wide installation.

### Using pip

```bash
pip install colcon-uv --break-system-packages
```

### Using uv

```bash
uv pip install --system --break-system-packages colcon-uv
```

### Install from GitHub (development version)

```bash
# Using pip
pip install git+https://github.com/nzlz/colcon-uv.git#subdirectory=colcon_uv --break-system-packages

# Using uv
uv pip install --system --break-system-packages git+https://github.com/nzlz/colcon-uv.git#subdirectory=colcon_uv
```

## Quick Start

For practical examples, see the test examples in this repository:

- **`ament_cmake`** for C++/Python hybrid packages [`tests/examples/uv_ament_cmake_example`](tests/examples/uv_ament_cmake_example/pyproject.toml)
- **`ament_python`** for pure Python packages [`tests/examples/uv_ament_python_example`](tests/examples/uv_ament_python_example/pyproject.toml)

## Configuration

Refer to [colcon_uv/README.md](colcon_uv/README.md#configuration)

## References

This project is inspired by and builds upon:

- [colcon-poetry-ros](https://github.com/UrbanMachine/colcon-poetry-ros) - Poetry integration for ROS packages
- [colcon-ros](https://github.com/colcon/colcon-ros/tree/colcon-python-project) - Experimental ROS support for colcon
- [uv](https://github.com/astral-sh/uv) - An extremely fast Python package installer and resolver
