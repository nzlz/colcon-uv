"""The build type a colcon-uv package asks for in ``[tool.colcon-uv-ros]``.

``python`` (the default) is a Python project: colcon-uv installs the package itself and its
dependencies into the package's venv. ``ament_cmake`` is a C++ (or mixed) ROS package built by
ament_cmake: colcon-uv only installs the package's PyPI dependencies into its venv, makes that
venv visible to CMake and hands the build to colcon-ros's ament_cmake task.
"""

from typing import Any, Dict

PYTHON = "python"
AMENT_CMAKE = "ament_cmake"
BUILD_TYPES = (PYTHON, AMENT_CMAKE)

# The colcon package type of an ament_cmake package whose PyPI dependencies colcon-uv installs.
# It starts with "ros." so colcon-ros still reads the package.xml dependencies.
AMENT_CMAKE_PACKAGE_TYPE = "ros.ament_cmake.uv"


class InvalidBuildTypeError(ValueError):
    """Raised for a build-type colcon-uv does not know."""


def build_type(pyproject_data: Dict[str, Any]) -> str:
    """The build type of a parsed pyproject.toml (``python`` when not set)."""
    config = pyproject_data.get("tool", {}).get("colcon-uv-ros", {})
    value = config.get("build-type", PYTHON)
    if value not in BUILD_TYPES:
        raise InvalidBuildTypeError(
            f"[tool.colcon-uv-ros] build-type must be one of {', '.join(BUILD_TYPES)}, "
            f"not {value!r}"
        )
    return value


def package_xml_name(path):
    """The <name> of the package.xml in directory ``path`` (None when it cannot be read)."""
    try:
        from catkin_pkg.package import parse_package

        return parse_package(str(path)).name
    except Exception:  # noqa: BLE001 - any manifest problem: fall back
        return None
