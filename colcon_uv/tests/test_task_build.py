"""Tests for UV build task."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch


class TestUvBuildTask(unittest.TestCase):
    """Test UV build task functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.task.uv.build import UvBuildTask

        self.task_class = UvBuildTask

    def test_task_initialization(self):
        """Test that build task can be initialized."""
        task = self.task_class()
        self.assertIsNotNone(task)
        self.assertTrue(hasattr(task, "build"))
        self.assertTrue(hasattr(task, "add_arguments"))

    def test_add_arguments(self):
        """Test that arguments are added to parser."""
        import argparse

        task = self.task_class()
        parser = argparse.ArgumentParser()
        task.add_arguments(parser=parser)

        # Should not raise an exception
        args = parser.parse_args([])
        self.assertIsNotNone(args)

        # Test with UV args
        args = parser.parse_args(["--uv-args", "verbose"])
        self.assertEqual(args.uv_args, ["verbose"])

    @patch("colcon_uv.task.uv.build.install_dependencies_from_descriptor")
    async def test_build_method(self, mock_install):
        """Test the main build method."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml
            pyproject_content = """
[project]
name = "test_package"

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()

            # Mock context
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            # Mock the internal methods
            task._add_data_files = AsyncMock(return_value=0)
            task._create_executable_symlinks = MagicMock()
            task._create_environment_hooks = MagicMock()

            # Test successful build
            await task.build()

            # Verify calls
            mock_install.assert_called_once()
            task._add_data_files.assert_called_once()
            task._create_executable_symlinks.assert_called_once()
            task._create_environment_hooks.assert_called_once()

    async def test_add_data_files_no_pyproject(self):
        """Test _add_data_files with no pyproject.toml."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            result = await task._add_data_files()
            self.assertEqual(result, 0)

    async def test_add_data_files_no_uv_config(self):
        """Test _add_data_files with no colcon-uv-ros section."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml without colcon-uv-ros section
            pyproject_content = """
[project]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            result = await task._add_data_files()
            self.assertEqual(result, 0)

    async def test_add_data_files_success(self):
        """Test _add_data_files with valid data files."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create source files
            launch_dir = temp_path / "launch"
            launch_dir.mkdir()
            (launch_dir / "test.launch.py").write_text("# Launch file")

            config_file = temp_path / "config.yaml"
            config_file.write_text("key: value")

            # Create pyproject.toml with data files
            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"
data-files = {
    "share/test_package/launch" = ["launch"],
    "share/test_package" = ["config.yaml"]
}
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            result = await task._add_data_files()
            self.assertEqual(result, 0)

            # Verify files were copied
            self.assertTrue(
                (
                    temp_path
                    / "install"
                    / "share"
                    / "test_package"
                    / "launch"
                    / "launch"
                    / "test.launch.py"
                ).exists()
            )
            self.assertTrue(
                (
                    temp_path / "install" / "share" / "test_package" / "config.yaml"
                ).exists()
            )

    async def test_add_data_files_invalid_config(self):
        """Test _add_data_files with invalid configuration."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml with invalid data-files (not a dict)
            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"
data-files = ["invalid"]
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            result = await task._add_data_files()
            self.assertEqual(result, 1)

    async def test_add_data_files_invalid_sources(self):
        """Test _add_data_files with invalid sources configuration."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml with invalid sources (not a list)
            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"
data-files = {
    "share/test_package" = "invalid_not_list"
}
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            result = await task._add_data_files()
            self.assertEqual(result, 1)

    def test_create_executable_symlinks_no_pyproject(self):
        """Test _create_executable_symlinks with no pyproject.toml."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            # Should not raise an exception
            task._create_executable_symlinks()

    def test_create_executable_symlinks_no_scripts(self):
        """Test _create_executable_symlinks with no scripts section."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml without scripts
            pyproject_content = """
[project]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            # Should not raise an exception
            task._create_executable_symlinks()

    def test_create_executable_symlinks_success(self):
        """Test _create_executable_symlinks with valid scripts."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            install_path = temp_path / "install"

            # Create venv with executable
            venv_bin = install_path / "venv" / "bin"
            venv_bin.mkdir(parents=True)
            test_executable = venv_bin / "test_script"
            test_executable.write_text("#!/usr/bin/env python3\nprint('test')")
            test_executable.chmod(0o755)

            # Create pyproject.toml with scripts
            pyproject_content = """
[project]
scripts = {"test_script" = "test_package.main:main"}
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(install_path)

            task._create_executable_symlinks()

            # Verify symlink was created
            ros_executable = install_path / "lib" / "test_package" / "test_script"
            self.assertTrue(ros_executable.is_symlink())
            self.assertEqual(ros_executable.readlink(), test_executable)

    @patch("colcon_core.environment.create_environment_scripts")
    @patch("colcon_core.environment.create_environment_hooks")
    @patch("colcon_core.shell.create_environment_hook")
    def test_create_environment_hooks(
        self, mock_create_hook, mock_create_hooks, mock_create_scripts
    ):
        """Test _create_environment_hooks method."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Setup mocks
            mock_create_hook.return_value = ["test_hook"]
            mock_create_hooks.return_value = ["hook1", "hook2"]

            task = self.task_class()
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            task._create_environment_hooks()

            # Verify all functions were called
            mock_create_hook.assert_called_once()
            mock_create_hooks.assert_called_once()
            mock_create_scripts.assert_called_once()


class TestUvTestTask(unittest.TestCase):
    """Test UV test task functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.task.uv.test import UvTestTask

        self.task_class = UvTestTask

    def test_task_initialization(self):
        """Test that test task can be initialized."""
        task = self.task_class()
        self.assertIsNotNone(task)
        self.assertTrue(hasattr(task, "test"))
        self.assertTrue(hasattr(task, "add_arguments"))

    def test_add_arguments(self):
        """Test that arguments are added to parser."""
        import argparse

        task = self.task_class()
        parser = argparse.ArgumentParser()
        task.add_arguments(parser=parser)

        # Should not raise an exception
        args = parser.parse_args([])
        self.assertIsNotNone(args)

    @patch("subprocess.run")
    async def test_test_method(self, mock_run):
        """Test the main test method."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml
            pyproject_content = """
[project]
name = "test_package"

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            task = self.task_class()

            # Mock context
            task.context = MagicMock()
            task.context.pkg = MagicMock()
            task.context.pkg.path = temp_path
            task.context.pkg.name = "test_package"
            task.context.args = MagicMock()
            task.context.args.install_base = str(temp_path / "install")

            # Mock successful subprocess run
            mock_run.return_value = MagicMock(returncode=0)

            # Test successful test run
            await task.test()

            # Verify subprocess was called
            mock_run.assert_called()


if __name__ == "__main__":
    unittest.main()
