"""Integration tests for standard ament_python build and install commands."""

import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest


@pytest.mark.parametrize("backend", ["hatchling.build", "setuptools.build_meta"])
def test_standard_install_survives_relocation(tmp_path, backend):
    source = tmp_path / "source"
    source.mkdir()
    (source / "package.xml").write_text(
        '<package format="3"><name>bundle_test</name><version>0.1.0</version>'
        "<description>Bundle test</description>"
        '<maintainer email="test@example.com">Test</maintainer><license>MIT</license>'
        "<export><build_type>ament_python</build_type></export></package>"
    )
    (source / "setup.py").write_text(
        "from colcon_uv.setuptools import setup\nsetup(py_modules=['bundle_test'])\n"
    )
    requirements = (
        '["hatchling"]'
        if backend == "hatchling.build"
        else '["setuptools", "colcon-uv"]'
    )
    (source / "pyproject.toml").write_text(
        f"[build-system]\nrequires = {requirements}\nbuild-backend = '{backend}'\n"
        '[project]\nname = "bundle_test"\nversion = "0.1.0"\n'
        '[project.scripts]\nbundle-test = "bundle_test:main"\n'
        '[tool.colcon-uv-ros.data-files]\n"share/bundle_test" = ["settings.yaml"]\n'
    )
    (source / "bundle_test.py").write_text('def main():\n    print("bundle works")\n')
    (source / "settings.yaml").write_text("bundled: true\n")
    environment = os.environ.copy()
    # Exercise the unreleased local package without fetching colcon-uv from PyPI.
    environment["UV_NO_BUILD_ISOLATION"] = "1"
    environment["http_proxy"] = "http://127.0.0.1:9/"
    environment["https_proxy"] = "https://127.0.0.1:9/"
    staging = tmp_path / "staging"
    subprocess.run(
        [
            sys.executable,
            "setup.py",
            "install",
            "--root",
            str(staging),
            "--prefix",
            "/opt/ros/test",
            "--install-data",
            "/opt/ros/test",
        ],
        cwd=source,
        env=environment,
        check=True,
        timeout=120,
    )
    # Wheel builds must not recursively bundle another copy of themselves.
    subprocess.run(
        [sys.executable, "setup.py", "bdist_wheel"],
        cwd=source,
        check=True,
        timeout=120,
    )
    with zipfile.ZipFile(next((source / "dist").glob("*.whl"))) as wheel:
        assert not any("venv/" in name for name in wheel.namelist())

    installed = tmp_path / "relocated"
    shutil.move(staging / "opt/ros/test", installed)
    shutil.rmtree(source)
    environment = {"PATH": "/usr/bin:/bin"}
    executable = installed / "lib/bundle_test/bundle-test"
    output = subprocess.check_output(
        [str(executable)], env=environment, cwd="/", text=True
    )
    assert output.strip() == "bundle works"
    assert list((installed / "lib/bundle_test").iterdir()) == [executable]
    assert (installed / "share/bundle_test/settings.yaml").is_file()
    assert (
        installed / "share/ament_index/resource_index/packages/bundle_test"
    ).is_file()
    python = installed / "lib/colcon_uv/bundle_test/venv/bin/python"
    module = subprocess.check_output(
        [str(python), "-c", "import bundle_test; print(bundle_test.__file__)"],
        env=environment,
        cwd="/",
        text=True,
    ).strip()
    assert Path(module).relative_to(installed)
