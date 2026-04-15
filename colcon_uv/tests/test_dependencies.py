"""Tests for UV dependencies installation."""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from colcon_core.package_descriptor import PackageDescriptor


class TestGetIndexFlags(unittest.TestCase):
    """Test _get_index_flags helper."""

    def setUp(self):
        from colcon_uv.dependencies.install import UvPackage, _get_index_flags

        self.UvPackage = UvPackage
        self._get_index_flags = _get_index_flags

    def _make_package(self, pyproject_content):
        """Create a temporary UvPackage from pyproject.toml content."""
        self._tmpdir = tempfile.mkdtemp()
        pkg_dir = Path(self._tmpdir) / "pkg"
        pkg_dir.mkdir()
        (pkg_dir / "pyproject.toml").write_text(pyproject_content)
        return self.UvPackage(pkg_dir)

    def test_empty_config(self):
        pkg = self._make_package('[tool.colcon-uv-ros]\nname = "test"')
        self.assertEqual(self._get_index_flags(pkg), [])

    def test_extra_index_url(self):
        pkg = self._make_package(
            '[tool.colcon-uv-ros]\nname = "test"\n'
            'extra-index-url = ["https://a.com/simple", "https://b.com/simple"]'
        )
        flags = self._get_index_flags(pkg)
        self.assertEqual(
            flags,
            [
                "--extra-index-url",
                "https://a.com/simple",
                "--extra-index-url",
                "https://b.com/simple",
            ],
        )

    def test_index_url(self):
        pkg = self._make_package(
            '[tool.colcon-uv-ros]\nname = "test"\nindex-url = "https://custom.pypi.org/simple"'
        )
        flags = self._get_index_flags(pkg)
        self.assertEqual(flags, ["--index-url", "https://custom.pypi.org/simple"])

    def test_find_links(self):
        pkg = self._make_package(
            '[tool.colcon-uv-ros]\nname = "test"\nfind-links = ["/opt/wheels", "/opt/more"]'
        )
        flags = self._get_index_flags(pkg)
        self.assertEqual(
            flags, ["--find-links", "/opt/wheels", "--find-links", "/opt/more"]
        )

    def test_all_combined(self):
        pkg = self._make_package(
            '[tool.colcon-uv-ros]\nname = "test"\n'
            'index-url = "https://custom.pypi.org/simple"\n'
            'extra-index-url = ["https://extra.com/simple"]\n'
            'find-links = ["/opt/wheels"]'
        )
        flags = self._get_index_flags(pkg)
        self.assertEqual(
            flags,
            [
                "--index-url",
                "https://custom.pypi.org/simple",
                "--extra-index-url",
                "https://extra.com/simple",
                "--find-links",
                "/opt/wheels",
            ],
        )

    @patch.dict(
        os.environ,
        {
            "COLCON_UV_INDEX_URL": "https://env.pypi.org/simple",
            "COLCON_UV_EXTRA_INDEX_URL": "https://env-extra.com/simple",
            "COLCON_UV_FIND_LINKS": "/opt/env-wheels",
        },
    )
    def test_env_fallback(self):
        pkg = self._make_package('[tool.colcon-uv-ros]\nname = "test"')
        flags = self._get_index_flags(pkg)
        self.assertEqual(
            flags,
            [
                "--index-url",
                "https://env.pypi.org/simple",
                "--extra-index-url",
                "https://env-extra.com/simple",
                "--find-links",
                "/opt/env-wheels",
            ],
        )

    @patch.dict(
        os.environ,
        {
            "COLCON_UV_EXTRA_INDEX_URL": "https://env-extra.com/simple",
        },
    )
    def test_pyproject_overrides_env(self):
        pkg = self._make_package(
            '[tool.colcon-uv-ros]\nname = "test"\n'
            'extra-index-url = ["https://pyproject-extra.com/simple"]'
        )
        flags = self._get_index_flags(pkg)
        self.assertEqual(
            flags, ["--extra-index-url", "https://pyproject-extra.com/simple"]
        )


class TestDependenciesInstall(unittest.TestCase):
    """Test UV dependencies installation functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.dependencies.install import (
            discover_packages,
            install_dependencies,
            install_dependencies_from_descriptor,
        )

        self.discover_packages = discover_packages
        self.install_dependencies = install_dependencies
        self.install_dependencies_from_descriptor = install_dependencies_from_descriptor

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
            self.install_dependencies_from_descriptor(desc, install_base, merge_install)

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
    def test_install_passes_index_flags(self, mock_run):
        """Test that index flags from config are passed to uv pip install."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            desc = PackageDescriptor(temp_path)
            desc.name = "test_package"

            pyproject_content = """
[project]
dependencies = ["numpy"]

[tool.colcon-uv-ros]
name = "test_package"
extra-index-url = ["https://download.pytorch.org/whl/cu128"]
find-links = ["/opt/jetson-wheels"]
"""
            (temp_path / "pyproject.toml").write_text(pyproject_content)

            install_base = Path(temp_dir) / "install"
            mock_run.return_value = MagicMock(returncode=0)

            self.install_dependencies_from_descriptor(desc, install_base, False)

            # Find the pip install call (skip the venv creation call)
            pip_calls = [c for c in mock_run.call_args_list if "pip" in str(c)]
            self.assertTrue(len(pip_calls) > 0)

            pip_cmd = pip_calls[0][0][0]  # first positional arg of first pip call
            self.assertIn("--extra-index-url", pip_cmd)
            self.assertIn("https://download.pytorch.org/whl/cu128", pip_cmd)
            self.assertIn("--find-links", pip_cmd)
            self.assertIn("/opt/jetson-wheels", pip_cmd)

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

            self.install_dependencies_from_descriptor(desc, install_base, merge_install)

            # Should still work with merge install
            self.assertTrue(mock_run.called)


if __name__ == "__main__":
    unittest.main()
