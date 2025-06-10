"""Command-line interface for colcon-uv."""

import argparse
import logging
import sys
from pathlib import Path

from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint

from .dependencies.install import discover_packages, install_dependencies


class UvVerb(VerbExtensionPoint):
    """UV command for colcon."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, "^1.0")

    def add_arguments(self, *, parser):  # noqa: D102
        subparsers = parser.add_subparsers(dest="uv_command", help="UV commands")

        # Install subcommand
        install_parser = subparsers.add_parser(
            "install", help="Install dependencies for UV-based packages"
        )
        install_parser.add_argument(
            "--base-paths",
            nargs="*",
            help="The base paths to recursively crawl for packages",
        )
        install_parser.add_argument(
            "--uv-args", nargs="*", help="Additional arguments to pass to UV"
        )

    def main(self, *, context):  # noqa: D102
        args = context.args
        logger = logging.getLogger("colcon.uv.cli")

        if not hasattr(args, "uv_command") or args.uv_command != "install":
            logger.error("Usage: colcon uv install [options]")
            return 1

        # Get base paths
        base_paths = args.base_paths
        if not base_paths:
            base_paths = [Path(".")]
        else:
            base_paths = [Path(p) for p in base_paths]

        # Discover UV packages using the install.py logic
        try:
            packages = discover_packages(base_paths)
        except SystemExit:
            logger.warning("No UV packages found")
            return 0

        # Install dependencies for each package using install.py logic
        install_base = Path("install")  # Default install base
        merge_install = False  # Default to separate installs

        for package in packages:
            try:
                install_dependencies(package, install_base, merge_install)
            except Exception as e:
                logger.error(
                    f"Failed to install dependencies for '{package.name}': {e}"
                )
                return 1

        return 0


def main():
    """Run the CLI."""
    parser = argparse.ArgumentParser(
        description="Install dependencies for UV-based packages"
    )
    parser.add_argument(
        "--base-paths",
        nargs="*",
        help="The base paths to recursively crawl for packages",
    )
    parser.add_argument(
        "--uv-args", nargs="*", help="Additional arguments to pass to UV"
    )

    args = parser.parse_args()

    # Set up logging
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s:%(name)s: %(message)s"
    )

    # Run the UV verb
    verb = UvVerb()
    return verb.main(context=argparse.Namespace(args=args))


if __name__ == "__main__":
    sys.exit(main())
