# colcon-uv

[![CI](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml/badge.svg)](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml)

A **colcon extension** for building and testing Python packages that use **[uv](https://github.com/astral-sh/uv)** for dependency management.

## Features

- **Fast Dependency Management**: Leverages UV's lightning-fast dependency resolution and installation
- **Modern Python Packaging**: Support for `pyproject.toml`-based packages following PEP 517/518 standards
- **ROS Integration**: Seamless integration with colcon build system and ROS package management
- **Dependency Isolation**: Prevents dependency conflicts between packages

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

### Package Source Configuration

Control where `uv pip install` fetches packages from. This is essential for platforms with custom-built wheels (e.g., NVIDIA Jetson with CUDA-specific torch builds) or private package indexes.

```toml
[tool.colcon-uv-ros]
name = "my_package"

# Override the default PyPI index (single string)
# index-url = "https://custom.pypi.org/simple"

# Additional package indexes (list)
extra-index-url = ["https://my-private.pypi.org/simple"]

# Local wheel directories or URLs (list)
find-links = ["/opt/jetson-wheels"]
```

When pyproject.toml keys are absent, the following environment variables are used as fallback:
- `COLCON_UV_INDEX_URL` — overrides default index
- `COLCON_UV_EXTRA_INDEX_URL` — comma-separated list of extra indexes
- `COLCON_UV_FIND_LINKS` — comma-separated list of find-links paths

Precedence: pyproject.toml > environment variables > uv defaults.

#### Jetson / Custom Platform Example

On platforms like NVIDIA Jetson, PyPI wheels for packages like `torch` are CPU-only. The Jetson needs GPU-specific builds. Combine `extra-site-packages` (to pre-seed existing builds) with `find-links` (for local wheel directories):

```toml
[tool.colcon-uv-ros]
name = "my_perception_package"

# Pre-seed the venv with packages from this path so uv skips them.
# Their dist-info is copied into the venv, and a .pth file makes
# Python resolve the actual modules at runtime.
extra-site-packages = ["/opt/venv/lib/python3.12/site-packages"]

# Local Jetson-built wheels for deps not pre-seeded above
find-links = ["/opt/jetson-wheels"]
```

Pin versions in `[project.dependencies]` to match your pre-seeded builds so uv doesn't resolve a different version from PyPI:

```toml
[project]
dependencies = [
    "torch==2.7.0",
    "torchvision==0.22.0",
    "ultralytics",
]
```
