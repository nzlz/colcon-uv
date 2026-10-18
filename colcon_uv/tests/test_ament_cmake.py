"""Tests for ament_cmake packages with PyPI dependencies (build-type = "ament_cmake")."""

import asyncio
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

from colcon_core.package_descriptor import PackageDescriptor

from colcon_uv.build_type import (
    AMENT_CMAKE_PACKAGE_TYPE,
    InvalidBuildTypeError,
    build_type,
)
from colcon_uv.package_identification.uv import UvPackageIdentification

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>test</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""

PYPROJECT = """
[project]
name = "{name}"
version = "0.1.0"
dependencies = ["casadi==3.8.1"]

[tool.colcon-uv-ros]
build-type = "{build_type}"
"""


def _write_package(path: Path, name="hybrid_pkg", build_type="ament_cmake", cmake=True):
    (path / "pyproject.toml").write_text(
        PYPROJECT.format(name=name.replace("_", "-"), build_type=build_type)
    )
    (path / "package.xml").write_text(PACKAGE_XML.format(name=name))
    if cmake:
        (path / "CMakeLists.txt").write_text("project(x)\n")


class TestBuildType(unittest.TestCase):
    """build_type() reads [tool.colcon-uv-ros].build-type."""

    def test_default_is_python(self):
        self.assertEqual(build_type({"tool": {"colcon-uv-ros": {}}}), "python")
        self.assertEqual(build_type({}), "python")

    def test_ament_cmake(self):
        data = {"tool": {"colcon-uv-ros": {"build-type": "ament_cmake"}}}
        self.assertEqual(build_type(data), "ament_cmake")

    def test_invalid(self):
        data = {"tool": {"colcon-uv-ros": {"build-type": "cmake"}}}
        with self.assertRaises(InvalidBuildTypeError):
            build_type(data)


class TestHybridIdentification(unittest.TestCase):
    """A build-type = "ament_cmake" package is identified as ros.ament_cmake.uv."""

    def _identify(self, path: Path) -> PackageDescriptor:
        desc = PackageDescriptor(path)
        UvPackageIdentification().identify(desc)
        return desc

    def test_identified(self):
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d))
            desc = self._identify(Path(d))
            self.assertEqual(desc.type, AMENT_CMAKE_PACKAGE_TYPE)
            # The ROS name comes from package.xml, not the (dashed) project name.
            self.assertEqual(desc.name, "hybrid_pkg")

    def test_without_cmakelists_not_claimed(self):
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d), cmake=False)
            self.assertIsNone(self._identify(Path(d)).type)

    def test_without_package_xml_not_claimed(self):
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d))
            (Path(d) / "package.xml").unlink()
            self.assertIsNone(self._identify(Path(d)).type)

    def test_invalid_build_type_not_claimed(self):
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d), build_type="meson")
            self.assertIsNone(self._identify(Path(d)).type)

    def test_python_build_type_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d), build_type="python")
            self.assertEqual(self._identify(Path(d)).type, "uv.python")

    def test_already_ament_cmake_not_overridden(self):
        """A package another extension identified as ros.ament_cmake is left alone."""
        with tempfile.TemporaryDirectory() as d:
            _write_package(Path(d))
            desc = PackageDescriptor(Path(d))
            desc.type = "ros.ament_cmake"
            UvPackageIdentification().identify(desc)
            self.assertEqual(desc.type, "ros.ament_cmake")


class TestRequirementsOnlyInstall(unittest.TestCase):
    """A hybrid package's venv gets its dependencies, never the package itself."""

    def test_install_command(self):
        from colcon_uv.dependencies.install import UvPackage, _install_requirements_only

        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            _write_package(path)
            with open(path / "pyproject.toml", "a") as fh:
                fh.write(
                    '\n[project.optional-dependencies]\nfast = ["numpy"]\n'
                    '\n[dependency-groups]\ndev = ["pytest"]\n'
                    '\n[tool.uv]\noverride-dependencies = ["numpy<3"]\n'
                )
            project = UvPackage(path)
            self.assertEqual(project.build_type, "ament_cmake")
            # The venv is keyed by the ROS name, not the PyPI project name
            self.assertEqual(project.name, "hybrid_pkg")
            with patch("colcon_uv.dependencies.install.subprocess.run") as run:
                run.return_value = MagicMock(returncode=0)
                _install_requirements_only(project, Path("/v/bin/python"), [])
            cmd = run.call_args.args[0]
            self.assertEqual(cmd[:3], ["uv", "--no-progress", "pip"])
            self.assertIn("-r", cmd)
            self.assertEqual(cmd[cmd.index("-r") + 1], "pyproject.toml")
            self.assertEqual(cmd[cmd.index("--python") + 1], "/v/bin/python")
            self.assertIn("--all-extras", cmd)
            self.assertEqual(cmd[cmd.index("--group") + 1], "dev")
            self.assertIn("--override", cmd)
            self.assertNotIn("-e", cmd)
            self.assertEqual(Path(run.call_args.kwargs["cwd"]), path)


class TestVenvSitePackages(unittest.TestCase):
    def test_found(self):
        from colcon_uv.dependencies.install import venv_site_packages

        with tempfile.TemporaryDirectory() as d:
            sp = Path(d) / "lib" / "python3.12" / "site-packages"
            sp.mkdir(parents=True)
            self.assertEqual(venv_site_packages(Path(d)), sp)

    def test_missing(self):
        from colcon_uv.dependencies.install import venv_site_packages

        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(venv_site_packages(Path(d)))


class TestVenvCmakeArgs(unittest.TestCase):
    def test_args(self):
        from colcon_uv.task.uv.ament_cmake import with_venv_cmake_args

        with tempfile.TemporaryDirectory() as d:
            sp = Path(d) / "site-packages"
            (sp / "cmeel.prefix").mkdir(parents=True)
            args = with_venv_cmake_args(
                ["-DFOO=1", "-DCMAKE_PREFIX_PATH=/a;/b"], Path(d), sp
            )
            self.assertEqual(args[0], "-DFOO=1")
            self.assertIn(f"-DCOLCON_UV_VENV={d}", args)
            self.assertIn(f"-DCOLCON_UV_SITE_PACKAGES={sp}", args)
            prefix = [a for a in args if a.startswith("-DCMAKE_PREFIX_PATH=")]
            self.assertEqual(
                prefix, [f"-DCMAKE_PREFIX_PATH={sp};{sp / 'cmeel.prefix'};/a;/b"]
            )

    def test_none_args(self):
        from colcon_uv.task.uv.ament_cmake import with_venv_cmake_args

        args = with_venv_cmake_args(None, Path("/v"), Path("/v/sp"))
        self.assertEqual(args[-1], "-DCMAKE_PREFIX_PATH=/v/sp")


class TestUvAmentCmakeBuildTask(unittest.TestCase):
    def test_add_arguments_does_not_clash(self):
        """colcon-ros's task registers --ament-cmake-args; ours must not again."""
        import argparse

        from colcon_ros.task.ament_cmake.build import AmentCmakeBuildTask

        from colcon_uv.task.uv.ament_cmake import UvAmentCmakeBuildTask

        parser = argparse.ArgumentParser()
        AmentCmakeBuildTask().add_arguments(parser=parser)
        UvAmentCmakeBuildTask().add_arguments(parser=parser)

    def test_build_installs_then_delegates(self):
        from colcon_ros.task.ament_cmake.build import AmentCmakeBuildTask

        from colcon_uv.task.uv.ament_cmake import UvAmentCmakeBuildTask

        with tempfile.TemporaryDirectory() as d:
            venv = Path(d) / "hybrid_pkg" / "venv"
            sp = venv / "lib" / "python3.12" / "site-packages"
            sp.mkdir(parents=True)
            task = UvAmentCmakeBuildTask()
            args = SimpleNamespace(install_base=d, cmake_args=None)
            task.context = SimpleNamespace(pkg=MagicMock(name="pkg"), args=args)
            with patch(
                "colcon_uv.task.uv.ament_cmake.install_dependencies_from_descriptor",
                return_value=venv,
            ) as install, patch.object(
                AmentCmakeBuildTask, "build", new=AsyncMock(return_value=0)
            ) as parent_build:
                rc = asyncio.run(task.build())
            self.assertEqual(rc, 0)
            install.assert_called_once_with(task.context.pkg, Path(d), False)
            parent_build.assert_awaited_once()
            self.assertIn(f"-DCOLCON_UV_VENV={venv}", args.cmake_args)
            self.assertIn(f"-DCMAKE_PREFIX_PATH={sp}", args.cmake_args)

    def test_build_fails_without_venv(self):
        from colcon_uv.task.uv.ament_cmake import UvAmentCmakeBuildTask

        task = UvAmentCmakeBuildTask()
        task.context = SimpleNamespace(
            pkg=MagicMock(), args=SimpleNamespace(install_base="/x", cmake_args=None)
        )
        with patch(
            "colcon_uv.task.uv.ament_cmake.install_dependencies_from_descriptor",
            return_value=None,
        ):
            self.assertEqual(asyncio.run(task.build()), 1)


class TestEntryPoints(unittest.TestCase):
    def test_registered(self):
        from importlib.metadata import entry_points

        build = {
            ep.name: ep.value for ep in entry_points(group="colcon_core.task.build")
        }
        test = {ep.name: ep.value for ep in entry_points(group="colcon_core.task.test")}
        self.assertEqual(
            build.get(AMENT_CMAKE_PACKAGE_TYPE),
            "colcon_uv.task.uv.ament_cmake:UvAmentCmakeBuildTask",
        )
        self.assertEqual(
            test.get(AMENT_CMAKE_PACKAGE_TYPE),
            "colcon_uv.task.uv.ament_cmake:UvAmentCmakeTestTask",
        )


if __name__ == "__main__":
    unittest.main()
