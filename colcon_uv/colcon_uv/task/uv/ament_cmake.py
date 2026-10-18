"""Build and test tasks for ament_cmake packages with PyPI dependencies.

A package with ``build-type = "ament_cmake"`` in ``[tool.colcon-uv-ros]`` is a normal ament_cmake
package (CMakeLists.txt + package.xml) whose pyproject.toml lists PyPI dependencies -- typically a
C++ library that ships as a wheel, with its shared libraries and a CMake package inside (CasADi,
cmeel-packaged libraries such as Eigen, ...). colcon-uv installs those dependencies into the
package's venv (``<install base>/<package>/venv``, as for a Python package), makes the venv
visible to CMake and then lets colcon-ros's ament_cmake task build, install and test the package.

CMake sees the venv through:

* ``CMAKE_PREFIX_PATH``, prepended with the venv's site-packages (``find_package(<name> CONFIG)``
  finds ``site-packages/<name>*/cmake/...``) and, when present, its ``cmeel.prefix``;
* ``-DCOLCON_UV_VENV=<venv>`` and ``-DCOLCON_UV_SITE_PACKAGES=<site-packages>``, for anything
  the package wants to locate itself.

At run time the venv's shared libraries are found through the RPATH the package links with (the
library directory a found CMake package reports), so no environment hook is added.
"""

from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import SkipExtensionException, satisfies_version
from colcon_core.task import TaskExtensionPoint

try:
    from colcon_ros.task.ament_cmake.build import AmentCmakeBuildTask
    from colcon_ros.task.ament_cmake.test import AmentCmakeTestTask
# Without colcon-ros the tasks skip themselves (see __init__).
except ImportError:
    AmentCmakeBuildTask = AmentCmakeTestTask = TaskExtensionPoint
    _HAVE_COLCON_ROS = False
else:
    _HAVE_COLCON_ROS = True

from colcon_uv.dependencies.install import (
    install_dependencies_from_descriptor,
    venv_site_packages,
)

logger = colcon_logger.getChild("colcon.uv.task.ament_cmake")


def cmake_prefix_entries(site_packages: Path):
    """The venv directories CMake should search for packages, in order."""
    entries = [site_packages]
    cmeel_prefix = site_packages / "cmeel.prefix"
    if cmeel_prefix.is_dir():
        entries.append(cmeel_prefix)
    return entries


_PREFIX_ARG = "-DCMAKE_PREFIX_PATH="


def with_venv_cmake_args(cmake_args, venv_path: Path, site_packages: Path):
    """``cmake_args`` (may be None) plus what points a build at its venv.

    The venv goes first in the ``CMAKE_PREFIX_PATH`` cache variable, which CMake searches before
    the environment variable ament fills in and before the system paths, so the venv's copy of a
    library wins over a system one. A ``-DCMAKE_PREFIX_PATH=`` the user passed is kept, after it.
    """
    prefixes = [str(p) for p in cmake_prefix_entries(site_packages)]
    args = list(cmake_args or [])
    user = [a for a in args if a.startswith(_PREFIX_ARG)]
    args = [a for a in args if not a.startswith(_PREFIX_ARG)]
    for a in user:
        prefixes += [p for p in a[len(_PREFIX_ARG) :].split(";") if p]
    return args + [
        # A package may ignore the COLCON_UV_* variables; don't warn about them.
        "--no-warn-unused-cli",
        f"-DCOLCON_UV_VENV={venv_path}",
        f"-DCOLCON_UV_SITE_PACKAGES={site_packages}",
        _PREFIX_ARG + ";".join(prefixes),
    ]


class UvAmentCmakeBuildTask(AmentCmakeBuildTask):
    """Install an ament_cmake package's PyPI dependencies, then build it with ament_cmake."""

    def __init__(self):  # noqa: D107
        if not _HAVE_COLCON_ROS:
            raise SkipExtensionException(
                "colcon-ros is needed to build ament_cmake packages"
            )
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, "^1.0")

    def add_arguments(self, *, parser):  # noqa: D102
        # colcon-ros's own ament_cmake task registers --ament-cmake-args already; registering
        # it twice is an argparse error.
        pass

    async def build(self):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        venv_path = install_dependencies_from_descriptor(
            pkg, Path(args.install_base), False
        )
        if venv_path is None:
            logger.error(
                f"{pkg.name}: no [tool.colcon-uv-ros] section, nothing to install"
            )
            return 1
        site_packages = venv_site_packages(venv_path)
        if site_packages is None:
            logger.error(f"{pkg.name}: the venv {venv_path} has no site-packages")
            return 1

        args.cmake_args = with_venv_cmake_args(
            args.cmake_args, venv_path, site_packages
        )
        logger.info(
            f"{pkg.name}: PyPI dependencies in {venv_path}, building with ament_cmake"
        )

        return await super().build()


class UvAmentCmakeTestTask(AmentCmakeTestTask):
    """Test an ament_cmake package with PyPI dependencies (ament_cmake's tests, unchanged)."""

    def __init__(self):  # noqa: D107
        if not _HAVE_COLCON_ROS:
            raise SkipExtensionException(
                "colcon-ros is needed to test ament_cmake packages"
            )
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, "^1.0")
