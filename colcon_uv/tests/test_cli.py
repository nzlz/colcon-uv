"""Tests for UV CLI functionality."""

import argparse
import unittest
from unittest.mock import MagicMock, patch


class TestUvVerb(unittest.TestCase):
    """Test UV verb functionality."""

    def setUp(self):
        """Set up test fixtures."""
        from colcon_uv.cli import UvVerb

        self.verb = UvVerb()

    def test_add_arguments(self):
        """Test that arguments are properly added to the parser."""
        parser = argparse.ArgumentParser()
        self.verb.add_arguments(parser=parser)

        # Parse test arguments (uv-args takes multiple arguments)
        args = parser.parse_args(["install", "--base-paths", "src"])

        self.assertEqual(args.uv_command, "install")
        self.assertEqual(args.base_paths, ["src"])

    def test_main_no_command(self):
        """Test main with no UV command."""
        context = MagicMock()
        context.args = argparse.Namespace()

        result = self.verb.main(context=context)
        self.assertEqual(result, 1)

    def test_main_wrong_command(self):
        """Test main with wrong UV command."""
        context = MagicMock()
        context.args = argparse.Namespace(uv_command="invalid")

        result = self.verb.main(context=context)
        self.assertEqual(result, 1)

    @patch("colcon_uv.cli.discover_packages")
    @patch("colcon_uv.cli.install_dependencies")
    def test_main_install_success(self, mock_install, mock_discover):
        """Test successful install command."""
        # Setup mocks
        mock_package = MagicMock()
        mock_package.name = "test_package"
        mock_discover.return_value = [mock_package]
        mock_install.return_value = None

        context = MagicMock()
        context.args = argparse.Namespace(
            uv_command="install", base_paths=["src"], uv_args=[]
        )

        result = self.verb.main(context=context)

        self.assertEqual(result, 0)
        mock_discover.assert_called_once()
        mock_install.assert_called_once()

    @patch("colcon_uv.cli.discover_packages")
    def test_main_install_no_packages(self, mock_discover):
        """Test install command with no packages found."""
        mock_discover.side_effect = SystemExit()

        context = MagicMock()
        context.args = argparse.Namespace(
            uv_command="install", base_paths=None, uv_args=[]
        )

        result = self.verb.main(context=context)
        self.assertEqual(result, 0)

    @patch("colcon_uv.cli.discover_packages")
    @patch("colcon_uv.cli.install_dependencies")
    def test_main_install_failure(self, mock_install, mock_discover):
        """Test install command with installation failure."""
        mock_package = MagicMock()
        mock_package.name = "test_package"
        mock_discover.return_value = [mock_package]
        mock_install.side_effect = Exception("Installation failed")

        context = MagicMock()
        context.args = argparse.Namespace(
            uv_command="install", base_paths=["src"], uv_args=[]
        )

        result = self.verb.main(context=context)
        self.assertEqual(result, 1)


class TestCliMain(unittest.TestCase):
    """Test CLI main function."""

    @patch("colcon_uv.cli.UvVerb")
    @patch("sys.argv", ["colcon-uv", "--base-paths", "src"])
    def test_main_function(self, mock_verb_class):
        """Test the main CLI function."""
        from colcon_uv.cli import main

        mock_verb = MagicMock()
        mock_verb.main.return_value = 0
        mock_verb_class.return_value = mock_verb

        result = main()

        self.assertEqual(result, 0)
        mock_verb.main.assert_called_once()


if __name__ == "__main__":
    unittest.main()
