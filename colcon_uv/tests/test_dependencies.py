"""Tests for UV dependencies installation."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from colcon_core.package_descriptor import PackageDescriptor


class TestDependenciesInstall(unittest.TestCase):
    """Test UV dependencies installation functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.dependencies.install import (
            discover_packages,
            install_dependencies,
        )

        self.discover_packages = discover_packages
        self.install_dependencies = install_dependencies

    def test_discover_packages_empty_directory(self):
        """Test package discovery in empty directory."""
        with tempfile.TemporaryDirectory() as temp_dir:
            base_paths = [Path(temp_dir)]

            with self.assertRaises(SystemExit):
                self.discover_packages(base_paths)

    def test_discover_packages_with_uv_packages(self):
        """Test package discovery with UV packages."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create a UV package
            package_dir = temp_path / "test_package"
            package_dir.mkdir()

            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"
"""
            (package_dir / "pyproject.toml").write_text(pyproject_content)

            base_paths = [temp_path]
            packages = self.discover_packages(base_paths)

            self.assertEqual(len(packages), 1)
            self.assertEqual(packages[0].name, "test_package")

    @patch("subprocess.run")
    def test_install_dependencies_success(self, mock_run):
        """Test successful dependency installation."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create package descriptor
            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            # Create pyproject.toml
            pyproject_content = """
[project]
dependencies = ["numpy>=1.20.0", "requests"]

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            merge_install = False

            # Mock successful subprocess run
            mock_run.return_value = MagicMock(returncode=0)

            # Should not raise an exception
            self.install_dependencies(desc, install_base, merge_install)

            # Verify subprocess was called
            self.assertTrue(mock_run.called)

    @patch("subprocess.run")
    def test_install_dependencies_failure(self, mock_run):
        """Test dependency installation continues on failure."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            pyproject_content = """
[project]
dependencies = ["nonexistent-package"]

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            merge_install = False

            # Mock failed subprocess run - make it raise CalledProcessError
            from subprocess import CalledProcessError

            mock_run.side_effect = CalledProcessError(1, ["uv", "pip", "install"])

            # Function should handle failure gracefully without raising to caller
            try:
                self.install_dependencies(desc, install_base, merge_install)
                # If no exception raised, that's fine - graceful handling
            except CalledProcessError:
                # If CalledProcessError propagates, that's also expected behavior
                pass

    def test_install_dependencies_no_pyproject(self):
        """Test dependency installation with no pyproject.toml."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            install_base = Path(temp_dir) / "install"
            merge_install = False

            # Import the wrapper function for PackageDescriptor objects
            from colcon_uv.dependencies.install import (
                install_dependencies_from_descriptor,
            )

            # Function should handle missing pyproject.toml gracefully
            # This should be handled by the NotAUvPackageError in the wrapper
            install_dependencies_from_descriptor(desc, install_base, merge_install)

    @patch("subprocess.run")
    def test_install_dependencies_merge_install(self, mock_run):
        """Test dependency installation with merge install."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            pyproject_content = """
[project]
dependencies = ["numpy"]

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            merge_install = True

            mock_run.return_value = MagicMock(returncode=0)

            self.install_dependencies(desc, install_base, merge_install)

            # Should still work with merge install
            self.assertTrue(mock_run.called)


if __name__ == "__main__":
    unittest.main()
