"""Install dependencies for UV packages."""

import argparse
import logging
import os
import shutil
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


def _preseed_extra_site_packages(project: UvPackage, venv_path: Path) -> None:
    """Pre-seed a venv with packages from extra site-packages paths.

    Copies *.dist-info directories into the venv so uv sees them as already
    installed, and writes a .pth file so Python resolves the actual modules
    at runtime.  This lets packages like Jetson-built torch stay in their
    original location (e.g. /opt/venv) without uv replacing them with
    CPU-only wheels from PyPI.
    """
    uv_ros_config = project.pyproject_data.get("tool", {}).get("colcon-uv-ros", {})
    extra_site_packages = uv_ros_config.get("extra-site-packages", [])
    if not extra_site_packages:
        return

    # Locate the venv site-packages (e.g. venv/lib/python3.12/site-packages)
    venv_site_dirs = list((venv_path / "lib").glob("python*/site-packages"))
    if not venv_site_dirs:
        logger.warning("Could not locate site-packages inside the venv")
        return

    venv_site = venv_site_dirs[0]
    pth_lines = []

    for extra_path_str in extra_site_packages:
        extra_path = Path(extra_path_str)
        if not extra_path.is_dir():
            logger.warning(f"extra-site-packages path not found: {extra_path}")
            continue

        # Copy dist-info so uv considers these packages already installed
        for dist_info in extra_path.glob("*.dist-info"):
            dest = venv_site / dist_info.name
            if not dest.exists():
                shutil.copytree(dist_info, dest)

        pth_lines.append(str(extra_path))

    if pth_lines:
        pth_file = venv_site / "colcon_uv_extra.pth"
        pth_file.write_text("\n".join(pth_lines) + "\n")
        logger.info(f"Pre-seeded venv with extra site-packages: {', '.join(pth_lines)}")


def _get_index_flags(project: UvPackage) -> List[str]:
    """Build uv pip install index flags from [tool.colcon-uv-ros] config.

    Reads index-url, extra-index-url, and find-links from the package's
    pyproject.toml [tool.colcon-uv-ros] section. Falls back to COLCON_UV_*
    environment variables when pyproject keys are absent.

    Precedence: pyproject.toml > env vars > uv defaults.
    """
    uv_ros_config = project.pyproject_data.get("tool", {}).get("colcon-uv-ros", {})
    flags: List[str] = []

    # index-url: single string, overrides default PyPI
    index_url = uv_ros_config.get("index-url") or os.environ.get("COLCON_UV_INDEX_URL")
    if index_url:
        flags.extend(["--index-url", index_url])

    # extra-index-url: list of additional indexes
    extra_index_urls = uv_ros_config.get("extra-index-url", [])
    if not extra_index_urls:
        env_val = os.environ.get("COLCON_UV_EXTRA_INDEX_URL", "")
        extra_index_urls = [u.strip() for u in env_val.split(",") if u.strip()]
    for url in extra_index_urls:
        flags.extend(["--extra-index-url", url])

    # find-links: list of local/remote wheel locations
    find_links = uv_ros_config.get("find-links", [])
    if not find_links:
        env_val = os.environ.get("COLCON_UV_FIND_LINKS", "")
        find_links = [u.strip() for u in env_val.split(",") if u.strip()]
    for link in find_links:
        flags.extend(["--find-links", link])

    if flags:
        logger.info(f"Using custom index flags: {' '.join(flags)}")

    return flags


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
    try:
        subprocess.run(
            ["uv", "venv", "--system-site-packages", str(venv_path)],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to create venv: {e.stderr}")
        raise

    # Pre-seed the venv with packages from extra-site-packages paths so that
    # uv sees them as already installed and does not re-fetch from PyPI.
    _preseed_extra_site_packages(project, venv_path)

    # Install dependencies and the package itself to the target venv
    # Use --python to specify the target venv's python
    python_exe = venv_path / "bin" / "python"
    index_flags = _get_index_flags(project)

    optional_deps = project.pyproject_data.get("project", {}).get(
        "optional-dependencies", {}
    )

    if optional_deps:
        extras = ",".join(optional_deps.keys())
        install_target = f"{project.path}[{extras}]"
        logger.info(f"Installing with optional dependencies: {extras}")
    else:
        install_target = str(project.path)

    try:
        subprocess.run(
            [
                "uv",
                "--no-progress",
                "pip",
                "install",
                *index_flags,
                "--python",
                str(python_exe),
                "-e",
                install_target,
            ],
            check=True,
            stdout=sys.stdout,
            stderr=sys.stderr,
            text=True,
        )
    except subprocess.CalledProcessError as e:
        # UV writes its errors to stderr, pass them through to user
        if e.stderr:
            sys.stderr.write(e.stderr)
            sys.stderr.flush()

        # Log simply without the exception details
        logger.error(f"Failed to install dependencies for {install_target}")

        # Re-raise without the traceback by using sys.exit
        # This prevents colcon from printing the full Python traceback
        sys.exit(1)

    # Additionally, install dependency groups (PEP 735) if present
    dependency_groups = project.pyproject_data.get("dependency-groups", {})

    if dependency_groups:
        group_names = list(dependency_groups.keys())
        logger.info(f"Installing dependency groups: {', '.join(group_names)}")

        cmd = [
            "uv",
            "--no-progress",
            "pip",
            "install",
            *index_flags,
            "--python",
            str(python_exe),
        ]
        for group in group_names:
            cmd.extend(["--group", group])
        cmd.append(str(project.path))

        try:
            subprocess.run(
                cmd, check=True, stdout=sys.stdout, stderr=sys.stderr, text=True
            )
        except subprocess.CalledProcessError as e:
            # UV writes its errors to stderr, pass them through to user
            if e.stderr:
                sys.stderr.write(e.stderr)
                sys.stderr.flush()

            # Log simply without the exception details
            logger.error(f"Failed to install dependency groups for {project.name}")

            # Re-raise without the traceback by using sys.exit
            # This prevents colcon from printing the full Python traceback
            sys.exit(1)


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
