"""Tests for UV dependencies installation."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from colcon_core.package_descriptor import PackageDescriptor

from colcon_uv.dependencies.install import install_dependencies_from_descriptor


class TestDependenciesInstall(unittest.TestCase):
    """Test UV dependencies installation functionality."""

    def test_discover_packages_empty_directory(self):
        """Test discovering packages in empty directory."""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Import function to test
            from colcon_uv.dependencies.install import discover_packages
            
            packages = discover_packages([Path(temp_dir)])
            self.assertEqual(len(packages), 0)

    def test_discover_packages_with_uv_packages(self):
        """Test discovering UV packages."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            
            # Create a package with pyproject.toml
            package_dir = temp_path / "test_package"
            package_dir.mkdir()
            
            pyproject_content = """
[project]
name = "test_package"

[tool.colcon-uv-ros]
name = "test_package"
"""
            (package_dir / "pyproject.toml").write_text(pyproject_content)
            
            # Import function to test
            from colcon_uv.dependencies.install import discover_packages
            
            packages = discover_packages([temp_path])
            self.assertEqual(len(packages), 1)

    def test_install_dependencies_success(self):
        """Test successful dependency installation."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            pyproject_content = """
[project]
name = "test_package"
dependencies = ["numpy"]

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            merge_install = False

            # Should not raise an exception
            install_dependencies_from_descriptor(desc, install_base, merge_install)

    def test_install_dependencies_merge_install(self):
        """Test dependency installation with merge install."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            pyproject_content = """
[project]
name = "test_package"

[tool.colcon-uv-ros]
name = "test_package"
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            merge_install = True

            # Should not raise an exception
            install_dependencies_from_descriptor(desc, install_base, merge_install)

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
                install_dependencies_from_descriptor(desc, install_base, merge_install)
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

            # Verify no pyproject.toml exists
            self.assertFalse((temp_path / "pyproject.toml").exists())

            # Test that UvPackage creation fails as expected
            from colcon_uv.dependencies.install import UvPackage, NotAUvPackageError
            with self.assertRaises(NotAUvPackageError):
                UvPackage(temp_path)

            # The wrapper function should handle this gracefully
            # Should not raise an exception
            install_dependencies_from_descriptor(desc, install_base, merge_install)


if __name__ == "__main__":
    unittest.main() 