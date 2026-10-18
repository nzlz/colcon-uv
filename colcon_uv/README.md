# colcon-uv

[![CI](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml/badge.svg)](https://github.com/nzlz/colcon-uv/actions/workflows/ci.yml)

A **colcon extension** for building and testing Python packages that use **[uv](https://github.com/astral-sh/uv)** for dependency management, and C++ `ament_cmake` packages whose dependencies come from PyPI.

## Features

- **Fast Dependency Management**: Leverages UV's lightning-fast dependency resolution and installation
- **Modern Python Packaging**: Support for `pyproject.toml`-based packages following PEP 517/518 standards
- **ROS Integration**: Seamless integration with colcon build system and ROS package management
- **Dependency Isolation**: Prevents dependency conflicts between packages
- **C++ packages with PyPI dependencies**: see [C++ (ament_cmake) packages](#c-ament_cmake-packages)

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

### C++ (ament_cmake) packages

A C++ package whose libraries come from PyPI (a wheel with shared libraries and a CMake package inside, e.g. `casadi`, or `cmeel-*` wheels such as `cmeel-eigen`) keeps its `CMakeLists.txt` and `package.xml` and adds a `pyproject.toml` that only lists those dependencies:

```toml
[project]
name = "my-cpp-package"
version = "0.1.0"
dependencies = ["casadi==3.8.1"]

[tool.colcon-uv-ros]
build-type = "ament_cmake"   # default: "python"
```

`build-type = "ament_cmake"` needs a `CMakeLists.txt` and a `package.xml` next to the `pyproject.toml`. The package is then a normal ament_cmake package (type `ros.ament_cmake.uv`; its ROS dependencies are read from `package.xml` as usual) and `colcon build`:

1. creates the package's venv at `install/<package>/venv` (merged and isolated installs alike) and installs only the `pyproject.toml` dependencies into it (all extras and dependency groups, `tool.uv.override-dependencies` and the index settings above apply), never the package itself;
2. builds, installs and tests the package with colcon-ros's ament_cmake tasks, with these CMake arguments added:
   - `CMAKE_PREFIX_PATH` starting with the venv's `site-packages` (and its `cmeel.prefix`, if any), so `find_package(<name> CONFIG)` finds the CMake packages inside the wheels before any system copy; a `-DCMAKE_PREFIX_PATH=...` you pass is kept after them;
   - `COLCON_UV_VENV` and `COLCON_UV_SITE_PACKAGES`, the venv and its `site-packages`, for anything the package locates itself.

`colcon uv install` installs the same dependencies without building.

At run time, the venv's shared libraries are found through the RPATH: link with the library directory the found package reports (CMake does this for imported targets in the build tree; for installed binaries set `INSTALL_RPATH`). No environment hook is added, so `LD_LIBRARY_PATH` is not changed.

```cmake
find_package(casadi CONFIG REQUIRED)  # site-packages/casadi/cmake in install/<package>/venv
get_filename_component(CASADI_LIB_DIR "${casadi_DIR}/.." ABSOLUTE)  # libcasadi.so lives there
target_link_libraries(my_node casadi)
set_target_properties(my_node PROPERTIES INSTALL_RPATH "${CASADI_LIB_DIR}")
```
