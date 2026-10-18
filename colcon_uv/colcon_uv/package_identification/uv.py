"""Identify packages that use UV for Python dependency management."""

import logging
import os

from colcon_core.package_identification import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version

from colcon_uv.build_type import (
    AMENT_CMAKE,
    AMENT_CMAKE_PACKAGE_TYPE,
    InvalidBuildTypeError,
    build_type,
    package_xml_name,
)

logger = logging.getLogger("colcon.uv.package_identification")


class UvPackageIdentification(PackageIdentificationExtensionPoint):
    """Identify packages that use UV for Python dependency management."""

    # High priority to supersede other identifiers like ament_python
    PRIORITY = 200

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def identify(self, desc):  # noqa: D102
        if desc.type is not None and desc.type not in (
            "uv.python",
            AMENT_CMAKE_PACKAGE_TYPE,
        ):
            return

        pyproject_toml = desc.path / "pyproject.toml"
        if not pyproject_toml.exists():
            return

        # Check if it's a UV package by looking for our custom section
        try:
            import tomli

            with open(pyproject_toml, "rb") as f:
                data = tomli.load(f)
                if "tool" in data and "colcon-uv-ros" in data["tool"]:
                    try:
                        kind = build_type(data)
                    except InvalidBuildTypeError as e:
                        logger.error(f"{pyproject_toml}: {e}")
                        return
                    if kind == AMENT_CMAKE:
                        self._identify_ament_cmake(desc, data)
                        return
                    desc.type = "uv.python"
                    # Get name from pyproject.toml if available, otherwise use directory name
                    if "project" in data and "name" in data["project"]:
                        desc.name = data["project"]["name"]
                    else:
                        desc.name = os.path.basename(desc.path)
                    return
        except ImportError:
            # If tomli is not available, we'll just check for the string
            with open(pyproject_toml) as f:
                content = f.read()
                if "[tool.colcon-uv-ros]" in content:
                    desc.type = "uv.python"
                    desc.name = os.path.basename(desc.path)
                    return

    def _identify_ament_cmake(self, desc, data):
        """An ament_cmake package that declares PyPI dependencies in its pyproject.toml.

        It needs the CMakeLists.txt and package.xml of any ament_cmake package. Its type starts
        with "ros." so colcon-ros reads the package.xml dependencies as usual; the name is the
        package.xml one (what ament and other packages use), else [project].name.
        """
        if (
            not (desc.path / "CMakeLists.txt").exists()
            or not (desc.path / "package.xml").exists()
        ):
            logger.error(
                f"{desc.path}: build-type = 'ament_cmake' needs a CMakeLists.txt and a "
                "package.xml next to the pyproject.toml"
            )
            return
        desc.type = AMENT_CMAKE_PACKAGE_TYPE
        if desc.name is None:
            desc.name = package_xml_name(desc.path) or data.get("project", {}).get(
                "name", os.path.basename(desc.path)
            )
