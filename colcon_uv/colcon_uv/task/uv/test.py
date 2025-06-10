"""Test task for UV-based Python packages."""

import subprocess
from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint

logger = colcon_logger.getChild("colcon.uv.task.test")


class UvTestTask(TaskExtensionPoint):
    """Test task for UV-based Python packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, "^1.0")

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            "--uv-args",
            nargs="*",
            metavar="*",
            type=str.lstrip,
            help="Pass arguments to UV. "
            "Arguments matching other options must be prefixed by a space,\n"
            'e.g. --uv-args " --help"',
        )

    async def test(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info("Testing UV package in '{args.path}'".format_map(locals()))

        # Get virtual environment path
        venv_path = Path(args.build_base) / "venv"
        if not venv_path.exists():
            logger.error(
                "Virtual environment not found. Please build the package first."
            )
            return

        # Run tests using pytest
        logger.info("Running tests...")
        subprocess.run([str(venv_path / "bin" / "pytest"), str(pkg.path)], check=True)
