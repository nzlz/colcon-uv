"""Tests for UV package identification."""

import tempfile
import unittest
from pathlib import Path

from colcon_core.package_descriptor import PackageDescriptor


class TestUvPackageIdentification(unittest.TestCase):
    """Test UV package identification functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.package_identification.uv import UvPackageIdentification

        self.identification = UvPackageIdentification()

    def test_identify_uv_package(self):
        """Test identification of UV packages."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml with colcon-uv-ros section
            pyproject_content = """
[build-system]
requires = ["hatchling"]

[project]
name = "test_uv_package"

[tool.colcon-uv-ros]
name = "test_uv_package"
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            desc = PackageDescriptor(temp_path)
            self.identification.identify(desc)

            # The identify method should set the package type to "uv.python"
            self.assertEqual(desc.type, "uv.python")
            self.assertEqual(desc.name, "test_uv_package")

    def test_identify_non_uv_package_with_pyproject(self):
        """Test that packages with pyproject.toml but no colcon-uv-ros section are not identified."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)

            # Create pyproject.toml without colcon-uv-ros section
            pyproject_content = """
[build-system]
requires = ["hatchling"]

[project]
name = "regular_package"
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            desc = PackageDescriptor(temp_path)
            self.identification.identify(desc)

            # Should remain None for non-UV packages
            self.assertIsNone(desc.type)

    def test_identify_package_without_pyproject_toml(self):
        """Test that packages without pyproject.toml are not identified."""
        with tempfile.TemporaryDirectory() as temp_dir:
            desc = PackageDescriptor(Path(temp_dir))
            self.identification.identify(desc)

            # Should remain None when no pyproject.toml exists
            self.assertIsNone(desc.type)


if __name__ == "__main__":
    unittest.main()
