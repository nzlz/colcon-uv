"""Setuptools commands for ament_python builds with bundled dependencies."""

import shutil
import subprocess
from pathlib import Path

from setuptools import setup as _setup
from setuptools.command.build import build as _build
from setuptools.command.install import install as _install

from colcon_uv.bundle import build_bundle


def _building_wheel(distribution):
    # Wheels remain ordinary Python packages, including the one installed into the venv.
    return any(
        command in distribution.commands
        for command in ("bdist_wheel", "editable_wheel")
    )


def _prepare_elf(path):
    # Strip before debhelper's debugedit processes auditwheel binaries.
    subprocess.run(["strip", "--strip-unneeded", str(path)], check=True)
    dynamic = subprocess.check_output(["readelf", "-d", str(path)], text=True)
    if "(NEEDED)" not in dynamic:
        return
    needed = subprocess.check_output(
        ["patchelf", "--print-needed", str(path)], text=True
    ).splitlines()
    if not any((path.parent / name).is_file() for name in needed):
        return
    rpath = subprocess.check_output(
        ["patchelf", "--print-rpath", str(path)], text=True
    ).strip()
    # Make sibling wheel libraries discoverable without changing existing search paths.
    if "$ORIGIN" not in rpath.split(":"):
        subprocess.run(
            [
                "patchelf",
                *(["--force-rpath"] if "(RPATH)" in dynamic else []),
                "--add-rpath",
                "$ORIGIN",
                str(path),
            ],
            check=True,
        )


class build(_build):
    """Build the Python modules and their standalone runtime environment."""

    def run(self):
        super().run()
        if _building_wheel(self.distribution):
            return
        output = Path(self.build_base) / "colcon_uv"
        if output.exists():
            shutil.rmtree(output)
        build_bundle(Path.cwd(), output)


class install(_install):
    """Install the bundle under the requested prefix, respecting staging roots."""

    def run(self):
        if _building_wheel(self.distribution):
            super().run()
            return
        if not self.skip_build:
            self.run_command("build")
        build_command = self.get_finalized_command("build")
        bundle = Path(build_command.build_base) / "colcon_uv"
        if not bundle.is_dir():
            raise FileNotFoundError(f"Run the build command first: {bundle}")
        destination = Path(self.install_data)
        shutil.copytree(bundle, destination, symlinks=True, dirs_exist_ok=True)
        for source in (bundle / "lib/colcon_uv").rglob("*"):
            path = destination / source.relative_to(bundle)
            if path.is_file() and not path.is_symlink():
                with path.open("rb") as stream:
                    is_elf = stream.read(4) == b"\x7fELF"
                if is_elf:
                    _prepare_elf(path)
        self._outputs = [
            str(destination / path.relative_to(bundle))
            for path in bundle.rglob("*")
            if path.is_file() or path.is_symlink()
        ]
        if self.record:
            paths = self._outputs
            if self.root:
                paths = ["/" + str(Path(path).relative_to(self.root)) for path in paths]
            Path(self.record).write_text("\n".join(paths) + "\n")

    def get_outputs(self):
        if _building_wheel(self.distribution):
            return super().get_outputs()
        return getattr(self, "_outputs", [])


def setup(**kwargs):
    """Use the bundle commands from an otherwise standard ament_python setup.py."""
    commands = {"build": build, "install": install}
    commands.update(kwargs.pop("cmdclass", {}))
    return _setup(cmdclass=commands, **kwargs)
