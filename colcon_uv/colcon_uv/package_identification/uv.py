"""Identify packages that use UV for Python dependency management."""

import logging
import os

from colcon_core.package_identification import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version

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
        if desc.type is not None and desc.type != "uv.python":
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
