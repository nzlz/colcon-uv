"""Build relocatable Python install trees for standard package builds."""

import os
import re
import shlex
import shutil
import tempfile
from pathlib import Path

from catkin_pkg.package import parse_package

from colcon_uv.build_type import PYTHON
from colcon_uv.dependencies.install import UvPackage, install_dependencies


def _project(path):
    project = UvPackage(Path(path).resolve())
    manifest = parse_package(str(project.path / "package.xml"))
    if project.build_type != PYTHON or manifest.get_build_type() != "ament_python":
        raise ValueError("Debian bundling only supports Python/ament_python packages")
    config = project.pyproject_data["tool"]["colcon-uv-ros"]
    if config.get("extra-site-packages"):
        raise ValueError(
            "extra-site-packages cannot be bundled; declare wheel dependencies instead"
        )
    return project, manifest


def _relative_path(value):
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Expected a path inside the package: {value}")
    return path


def build_bundle(path, output):
    """Build a source-independent install tree for Debian staging."""
    project, manifest = _project(path)
    output = Path(output).resolve()
    if output.exists():
        raise ValueError(
            f"Output already exists; remove it before rebuilding: {output}"
        )
    config = project.pyproject_data["tool"]["colcon-uv-ros"]
    data_files = config.get("data-files", {})
    for destination, sources in data_files.items():
        _relative_path(destination)
        if not isinstance(sources, list):
            raise ValueError(f"data-files.{destination} must be an array")
        for source in sources:
            source_path = project.path / _relative_path(source)
            if not source_path.exists():
                raise ValueError(f"Missing data file: {source_path}")
    scripts = project.pyproject_data.get("project", {}).get("scripts", {})
    for script in scripts:
        if not re.fullmatch(r"[a-zA-Z0-9_.-]+", script) or script in (
            ".",
            "..",
            "venv",
        ):
            raise ValueError(f"Unsupported console script name: {script}")
    lib_dir = output / "lib" / manifest.name
    venv_base = output / "lib/colcon_uv" / manifest.name
    environment = os.environ.copy()
    # Dependency bundling needs indexes; preserve proxies configured by the caller.
    for name, value in (
        ("http_proxy", "http://127.0.0.1:9/"),
        ("https_proxy", "https://127.0.0.1:9/"),
    ):
        if environment.get(name) == value:
            del environment[name]
    # pybuild's user distutils config must not leak into isolated wheel builds.
    with tempfile.TemporaryDirectory(prefix="colcon-uv-home-") as home:
        environment["HOME"] = home
        venv = install_dependencies(
            project, venv_base, True, bundle=True, subprocess_env=environment
        )
    for destination, sources in data_files.items():
        dest = output / destination
        dest.mkdir(parents=True, exist_ok=True)
        for source in sources:
            source_path = project.path / source
            target = dest / source_path.name
            if source_path.is_dir():
                shutil.copytree(source_path, target, dirs_exist_ok=True)
            else:
                shutil.copy2(source_path, target)
    share = output / "share" / manifest.name
    share.mkdir(parents=True, exist_ok=True)
    shutil.copy2(project.path / "package.xml", share / "package.xml")
    marker = output / "share/ament_index/resource_index/packages" / manifest.name
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()
    lib_dir.mkdir(parents=True, exist_ok=True)
    for script in scripts:
        if not (venv / "bin" / script).is_file():
            raise ValueError(f"Console script was not installed: {script}")
        wrapper = lib_dir / script
        wrapper.write_text(
            "#!/bin/sh\n"
            'script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)\n'
            f'exec "$script_dir/../colcon_uv/{manifest.name}/venv/bin/"'
            f'{shlex.quote(script)} "$@"\n'
        )
        wrapper.chmod(0o755)
