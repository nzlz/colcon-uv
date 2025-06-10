"""Install dependencies for UV packages."""

import argparse
import logging
import subprocess
import sys
from pathlib import Path
from typing import List

import tomli

logger = logging.getLogger("colcon.uv.dependencies")


class NotAUvPackageError(Exception):
    """Raised when a directory is not a UV package."""

    pass


class UvPackage:
    """Represents a UV package."""

    def __init__(self, path: Path, logger=None):
        """Initialize UV package."""
        self.path = path
        self.logger = logger or logging.getLogger(__name__)

        self.pyproject_file = path / "pyproject.toml"
        if not self.pyproject_file.exists():
            raise NotAUvPackageError(f"No pyproject.toml found in {path}")

        # Load pyproject.toml
        with open(self.pyproject_file, "rb") as f:
            self.pyproject_data = tomli.load(f)

        # Check if it's a UV package
        if (
            "tool" not in self.pyproject_data
            or "colcon-uv-ros" not in self.pyproject_data["tool"]
        ):
            raise NotAUvPackageError(
                f"No [tool.colcon-uv-ros] section found in {self.pyproject_file}"
            )

        # Get package name
        if (
            "project" in self.pyproject_data
            and "name" in self.pyproject_data["project"]
        ):
            self.name = self.pyproject_data["project"]["name"]
        else:
            self.name = path.name


def main():
    """Main entry point for UV dependency installation."""
    args = _parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s:%(name)s: %(message)s",
    )

    for project in discover_packages(args.base_paths):
        logger.info(f"Installing dependencies for {project.path.name}...")
        install_dependencies(project, args.install_base, args.merge_install)

    logger.info("Dependencies installed!")


def discover_packages(base_paths: List[Path]) -> List[UvPackage]:
    """Discover UV packages in the given base paths."""
    projects: List[UvPackage] = []

    potential_packages = []
    for path in base_paths:
        potential_packages += list(path.glob("*"))

    for path in potential_packages:
        if path.is_dir():
            try:
                project = UvPackage(path)
            except NotAUvPackageError:
                continue
            else:
                projects.append(project)

    if len(projects) == 0:
        base_paths_str = ", ".join([str(p) for p in base_paths])
        logger.error(
            f"No UV packages were found in the following paths: {base_paths_str}"
        )
        sys.exit(1)

    return projects


def install_dependencies(
    project: UvPackage, install_base: Path, merge_install: bool
) -> None:
    """Install dependencies for a UV package using UV."""
    # Handle both contexts:
    # 1. Direct install: install_base = /install, need to add package name
    # 2. Build task: install_base = /install/package_name, already included

    if not merge_install:
        # Check if install_base already ends with the package name
        if install_base.name != project.name:
            install_base /= project.name

    # Create the install directory first
    install_base.mkdir(parents=True, exist_ok=True)

    # Venv path - this should be /install/PACKAGE_NAME/venv/
    venv_path = install_base / "venv"

    # Create virtual environment at the target location with system packages access
    # --system-site-packages is needed because ROS 2 packages like rclpy are installed
    # system-wide (not available on PyPI) and our nodes need access to them
    subprocess.run(["uv", "venv", "--system-site-packages", str(venv_path)], check=True)

    # Install dependencies and the package itself to the target venv
    # Use --python to specify the target venv's python
    python_exe = venv_path / "bin" / "python"
    subprocess.run(
        ["uv", "pip", "install", "--python", str(python_exe), "-e", str(project.path)],
        check=True,
    )


def install_dependencies_from_descriptor(
    pkg_descriptor, install_base: Path, merge_install: bool
):
    """Install dependencies from a PackageDescriptor object.

    This is a convenience function for use by colcon build tasks.
    """
    try:
        uv_package = UvPackage(pkg_descriptor.path)
        install_dependencies(uv_package, install_base, merge_install)
    except NotAUvPackageError as e:
        # Skip packages that aren't UV packages
        logger.debug(f"Skipping non-UV package {pkg_descriptor.name}: {e}")
        return


def _parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Searches for UV packages and installs their dependencies "
        "to a configurable install base"
    )

    parser.add_argument(
        "--base-paths",
        nargs="+",
        type=Path,
        default=[Path.cwd()],
        help="The paths to start looking for UV projects in. Defaults to the "
        "current directory.",
    )

    parser.add_argument(
        "--install-base",
        type=Path,
        default=Path("install"),
        help="The base path for all install prefixes (default: install)",
    )

    parser.add_argument(
        "--merge-install",
        action="store_true",
        help="Merge all install prefixes into a single location",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="If provided, debug logs will be printed",
    )

    return parser.parse_args()


if __name__ == "__main__":
    main()
