"""Build task for UV-based Python packages."""

import shutil
from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint

from colcon_uv.dependencies.install import install_dependencies_from_descriptor

logger = colcon_logger.getChild("colcon.uv.task.build")


class UvBuildTask(TaskExtensionPoint):
    """Build task for UV-based Python packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, "^1.0")

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            "--uv-args",
            nargs="*",
            metavar="*",
            type=str.lstrip,
            help="Pass arguments to UV. "
            "Arguments matching other options must be prefixed by a space,\n"
            'e.g. --uv-args " --help"',
        )

    async def build(self, *, additional_hooks=None):
        pkg = self.context.pkg
        args = self.context.args

        # UV sync handles EVERYTHING - deps + package installation
        logger.info("Syncing dependencies and installing package...")
        install_dependencies_from_descriptor(pkg, Path(args.install_base), False)

        # Handle ROS-specific stuff (data files, environment hooks, etc.)
        return_code = await self._add_data_files()
        if return_code != 0:
            return return_code

        # Create executable symlinks for ROS 2
        self._create_executable_symlinks()

        # Add environment hooks for ROS
        self._create_environment_hooks()

    async def _add_data_files(self) -> int:
        """Install data files based on the [tool.colcon-uv-ros.data-files] table."""
        pkg = self.context.pkg
        args = self.context.args

        pyproject_toml = pkg.path / "pyproject.toml"

        try:
            import tomli

            with open(pyproject_toml, "rb") as f:
                data = tomli.load(f)

            if "tool" not in data or "colcon-uv-ros" not in data["tool"]:
                return 0

            uv_config = data["tool"]["colcon-uv-ros"]
            if "data-files" not in uv_config:
                return 0

            data_files = uv_config["data-files"]
        except (ImportError, FileNotFoundError, KeyError):
            return 0

        if not isinstance(data_files, dict):
            logger.error("data-files must be a table")
            return 1

        for destination, sources in data_files.items():
            if not isinstance(sources, list):
                logger.error(f"Field '{destination}' in data-files must be an array")
                return 1

            dest_path = Path(args.install_base) / destination
            dest_path.mkdir(parents=True, exist_ok=True)

            for source in sources:
                source_path = pkg.path / Path(source)
                if source_path.exists():
                    if source_path.is_dir():
                        shutil.copytree(
                            source_path,
                            dest_path / source_path.name,
                            dirs_exist_ok=True,
                        )
                    else:
                        shutil.copy2(source_path, dest_path)

        return 0

    def _create_executable_symlinks(self):
        """Create symlinks for executables in the standard ROS 2 location."""
        pkg = self.context.pkg
        args = self.context.args

        # Read pyproject.toml to get executable entry points
        pyproject_toml = pkg.path / "pyproject.toml"
        if not pyproject_toml.exists():
            return

        try:
            import tomli

            with open(pyproject_toml, "rb") as f:
                data = tomli.load(f)

            if "project" not in data or "scripts" not in data["project"]:
                return

            scripts = data["project"]["scripts"]
        except (ImportError, FileNotFoundError, KeyError):
            return

        # Create lib/package_name directory for executables
        lib_dir = Path(args.install_base) / "lib" / pkg.name
        lib_dir.mkdir(parents=True, exist_ok=True)

        # Create symlinks for each executable
        venv_bin = Path(args.install_base) / "venv" / "bin"

        for script_name, _entry_point in scripts.items():
            venv_executable = venv_bin / script_name
            ros_executable = lib_dir / script_name

            if venv_executable.exists():
                # Remove existing symlink if it exists
                if ros_executable.exists() or ros_executable.is_symlink():
                    ros_executable.unlink()

                # Create symlink
                ros_executable.symlink_to(venv_executable)
                logger.info(
                    f"Created executable symlink: {ros_executable} -> {venv_executable}"
                )

    def _create_environment_hooks(self):
        """Create ROS environment hooks."""
        from colcon_core.environment import (
            create_environment_hooks,
            create_environment_scripts,
        )
        from colcon_core.shell import create_environment_hook

        pkg = self.context.pkg
        args = self.context.args

        additional_hooks = create_environment_hook(
            "ament_prefix_path",
            Path(args.install_base) / pkg.name,
            pkg.name,
            "AMENT_PREFIX_PATH",
            "",
            mode="prepend",
        )

        hooks = create_environment_hooks(Path(args.install_base) / pkg.name, pkg.name)
        create_environment_scripts(
            pkg, args, default_hooks=list(hooks), additional_hooks=additional_hooks
        )
