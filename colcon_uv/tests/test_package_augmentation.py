"""Tests for UV package augmentation."""

import tempfile
import unittest
from pathlib import Path

from colcon_core.package_descriptor import PackageDescriptor


class TestUvPackageAugmentation(unittest.TestCase):
    """Test UV package augmentation functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.package_augmentation.uv import UvPackageAugmentation

        self.augmentation = UvPackageAugmentation()

    def test_augment_uv_package_with_all_dependencies(self):
        """Test that UV packages are properly augmented with all dependency types."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"

[tool.colcon-uv-ros.dependencies]
depend = ["rclpy", "sensor_msgs"]
build_depend = ["ament_cmake", "rosidl_default_generators"]
exec_depend = ["std_msgs", "geometry_msgs"]
test_depend = ["pytest", "ament_lint_auto"]

[tool.colcon-uv-ros.data-files]
"share/ament_index/resource_index/packages" = ["resource/test_package"]
"share/test_package" = ["package.xml", "launch"]
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            # Create test files
            (temp_path / "resource").mkdir()
            (temp_path / "resource" / "test_package").touch()
            (temp_path / "package.xml").touch()
            (temp_path / "launch").mkdir()

            # Create package descriptor
            desc = PackageDescriptor(temp_path)
            desc.type = "uv"

            # Augment the package
            self.augmentation.augment_package(desc)

            # Verify augmentation worked
            self.assertEqual(desc.name, "test_package")
            self.assertIn("build", desc.dependencies)
            self.assertIn("run", desc.dependencies)
            self.assertIn("test", desc.dependencies)

            # Check that 'depend' items are in both build and exec
            build_deps = {str(dep) for dep in desc.dependencies["build"]}
            run_deps = {str(dep) for dep in desc.dependencies["run"]}
            test_deps = {str(dep) for dep in desc.dependencies["test"]}

            self.assertIn("rclpy", build_deps)
            self.assertIn("rclpy", run_deps)
            self.assertIn("ament_cmake", build_deps)
            self.assertIn("std_msgs", run_deps)
            self.assertIn("pytest", test_deps)

    def test_augment_uv_package_minimal_config(self):
        """Test augmentation with minimal configuration."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            pyproject_content = """
[tool.colcon-uv-ros]
name = "minimal_package"
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            desc = PackageDescriptor(temp_path)
            desc.type = "uv"

            self.augmentation.augment_package(desc)
            self.assertEqual(desc.name, "minimal_package")

    def test_augment_uv_package_with_data_files_directory(self):
        """Test data files handling with directories."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            pyproject_content = """
[tool.colcon-uv-ros]
name = "test_package"

[tool.colcon-uv-ros.data-files]
"share/test_package" = ["config", "launch"]
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            # Create test directories
            (temp_path / "config").mkdir()
            (temp_path / "config" / "test.yaml").touch()
            (temp_path / "launch").mkdir()
            (temp_path / "launch" / "test.launch.py").touch()

            desc = PackageDescriptor(temp_path)
            desc.type = "uv"

            self.augmentation.augment_package(desc)
            self.assertEqual(desc.name, "test_package")

    def test_non_uv_package_ignored(self):
        """Test that non-UV packages are not augmented."""
        with tempfile.TemporaryDirectory() as temp_dir:
            desc = PackageDescriptor(Path(temp_dir))
            desc.type = "cmake"

            original_deps = dict(desc.dependencies)
            original_name = desc.name

            self.augmentation.augment_package(desc)

            self.assertEqual(desc.dependencies, original_deps)
            self.assertEqual(desc.name, original_name)

    def test_package_without_pyproject_toml(self):
        """Test UV package without pyproject.toml file."""
        with tempfile.TemporaryDirectory() as temp_dir:
            desc = PackageDescriptor(Path(temp_dir))
            desc.type = "uv"

            original_deps = dict(desc.dependencies)

            self.augmentation.augment_package(desc)

            # Should not crash, just not augment anything
            self.assertEqual(desc.dependencies, original_deps)

    def test_package_without_colcon_uv_ros_section(self):
        """Test pyproject.toml without colcon-uv-ros section."""
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            pyproject_content = """
[build-system]
requires = ["hatchling"]

[project]
name = "regular_package"
"""
            pyproject_file = temp_path / "pyproject.toml"
            pyproject_file.write_text(pyproject_content)

            desc = PackageDescriptor(temp_path)
            desc.type = "uv"

            original_deps = dict(desc.dependencies)

            self.augmentation.augment_package(desc)

            # Should not crash, just not augment anything
            self.assertEqual(desc.dependencies, original_deps)


if __name__ == "__main__":
    unittest.main()
