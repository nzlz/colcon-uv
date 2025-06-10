"""Augment packages that use UV for Python dependency management."""

import logging
import shutil
from pathlib import Path

from colcon_core.package_augmentation import PackageAugmentationExtensionPoint
from colcon_core.package_augmentation.python import create_dependency_descriptor
from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.plugin_system import satisfies_version

logger = logging.getLogger("colcon.uv.package_augmentation")


class UvPackageAugmentation(PackageAugmentationExtensionPoint):
    """Augment packages that use UV for Python dependency management."""

    _TOOL_SECTION = "tool"
    _COLCON_UV_ROS_SECTION = "colcon-uv-ros"
    _DEPENDENCIES_SECTION = "dependencies"
    _DATA_FILES_SECTION = "data-files"
    _DEPEND_LIST = "depend"
    _BUILD_DEPEND_LIST = "build_depend"
    _EXEC_DEPEND_LIST = "exec_depend"
    _TEST_DEPEND_LIST = "test_depend"
    _PACKAGE_BUILD_CATEGORY = "build"
    _PACKAGE_EXEC_CATEGORY = "run"
    _PACKAGE_TEST_CATEGORY = "test"

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION, "^1.0"
        )

    def augment_package(
        self, desc: PackageDescriptor, *, additional_argument_names=None
    ):  # noqa: D102
        if desc.type != "uv":
            return

        # Read pyproject.toml to get package metadata
        pyproject_toml = desc.path / "pyproject.toml"
        if not pyproject_toml.exists():
            return

        try:
            import tomli

            with open(pyproject_toml, "rb") as f:
                data = tomli.load(f)
                if (
                    self._TOOL_SECTION in data
                    and self._COLCON_UV_ROS_SECTION in data[self._TOOL_SECTION]
                ):
                    uv_config = data[self._TOOL_SECTION][self._COLCON_UV_ROS_SECTION]

                    # Set package name if not already set
                    if not desc.name and "name" in uv_config:
                        desc.name = uv_config["name"]

                    # Handle dependencies
                    if self._DEPENDENCIES_SECTION in uv_config:
                        deps = uv_config[self._DEPENDENCIES_SECTION]

                        # Build dependencies
                        if self._BUILD_DEPEND_LIST in deps:
                            build_deps = set(deps[self._BUILD_DEPEND_LIST])
                        else:
                            build_deps = set()

                        # Exec dependencies
                        if self._EXEC_DEPEND_LIST in deps:
                            exec_deps = set(deps[self._EXEC_DEPEND_LIST])
                        else:
                            exec_deps = set()

                        # Test dependencies
                        if self._TEST_DEPEND_LIST in deps:
                            test_deps = set(deps[self._TEST_DEPEND_LIST])
                        else:
                            test_deps = set()

                        # Handle general dependencies (add to both build and exec)
                        if self._DEPEND_LIST in deps:
                            depends = deps[self._DEPEND_LIST]
                            build_deps.update(depends)
                            exec_deps.update(depends)

                        # Set dependencies in descriptor
                        desc.dependencies[self._PACKAGE_BUILD_CATEGORY] = {
                            create_dependency_descriptor(dep) for dep in build_deps
                        }
                        desc.dependencies[self._PACKAGE_EXEC_CATEGORY] = {
                            create_dependency_descriptor(dep) for dep in exec_deps
                        }
                        desc.dependencies[self._PACKAGE_TEST_CATEGORY] = {
                            create_dependency_descriptor(dep) for dep in test_deps
                        }

                    # Handle data files
                    if self._DATA_FILES_SECTION in uv_config:
                        data_files = uv_config[self._DATA_FILES_SECTION]
                        for dest_dir, files in data_files.items():
                            # Create destination directory if it doesn't exist
                            dest_path = Path(dest_dir)
                            dest_path.mkdir(parents=True, exist_ok=True)

                            # Copy each file
                            for file in files:
                                src_path = desc.path / file
                                if src_path.is_file():
                                    shutil.copy2(src_path, dest_path / src_path.name)
                                elif src_path.is_dir():
                                    shutil.copytree(
                                        src_path,
                                        dest_path / src_path.name,
                                        dirs_exist_ok=True,
                                    )
        except ImportError:
            # If tomli is not available, we'll just use basic metadata
            pass
