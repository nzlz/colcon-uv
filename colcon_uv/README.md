# colcon-uv

[![CI](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml/badge.svg)](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml)

A **colcon extension** for building and testing Python packages that use **[uv](https://github.com/astral-sh/uv)** for dependency management.

## Features

- **Fast Dependency Management**: Leverages UV's lightning-fast dependency resolution and installation
- **Modern Python Packaging**: Support for `pyproject.toml`-based packages following PEP 517/518 standards
- **ROS Integration**: Seamless integration with colcon build system and ROS package management
- **Dependency Isolation**: Prevents dependency conflicts between packages

## Installation

```bash
pip install colcon-uv
```

## Configuration

### Data Files

Similar to [colcon-poetry-ros](https://github.com/UrbanMachine/colcon-poetry-ros), you can specify data files using the `[tool.colcon-uv-ros.data-files]` section:

```toml
[tool.colcon-uv-ros.data-files]
"share/ament_index/resource_index/packages" = ["resource/{package_name}"]
"share/{package_name}" = ["package.xml", "launch/", "config/"]
"lib/{package_name}" = ["scripts/"]
```

**Required entries** for all ROS packages:

```toml
[tool.colcon-uv-ros.data-files]
"share/ament_index/resource_index/packages" = ["resource/{package_name}"]
"share/{package_name}" = ["package.xml"]
```

### Package Dependencies

Specify package dependencies for build ordering and to use system libraries (fetched from system paths, not installed in virtual environment):

```toml
[tool.colcon-uv-ros.dependencies]
depend = ["rclpy", "geometry_msgs"]  # System packages (adds to both build_depend and exec_depend)
build_depend = ["bar_package"]       # Build-time only dependency
exec_depend = ["std_msgs"]           # Runtime system library
test_depend = ["qux_package"]        # Test-time only dependency
```

**Important**: ROS system libraries like `rclpy`, `geometry_msgs`, `std_msgs`, etc. should be listed here so they are resolved from the system installation rather than being installed into the virtual environment.
