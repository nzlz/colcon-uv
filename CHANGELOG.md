# Changelog

## Unreleased

- Packaging follows colcon/template-package: metadata and entry points move from `pyproject.toml` (hatchling) to `setup.cfg` (setuptools), and `stdeb.cfg` / `publish-python.yaml` describe the Debian package `python3-colcon-uv` for packages.ros.org. The PyPI wheel has the same files, dependencies and entry points as before.
- Package metadata now declares the MIT license, matching `LICENSE` (it said BSD-3-Clause). README: license section with an attribution note for colcon-poetry-ros.
- `colcon list` and `colcon build` no longer copy each package's `data-files` into a `share/` directory under the current directory; package augmentation did that on discovery. The build task still installs them into the install space.
- When the `uv` executable is not on `PATH`, the build and `colcon uv install` stop with an error that says how to install uv, instead of a `FileNotFoundError` traceback.

## 0.5.1

- README: no longer marked experimental; acknowledgments section removed. No code changes.

## 0.5.0

- C++ packages with PyPI dependencies: `build-type = "ament_cmake"` in `[tool.colcon-uv-ros]` makes an ament_cmake package (CMakeLists.txt + package.xml) take its PyPI dependencies from its own venv (`install/<package>/venv`). colcon-uv installs the dependencies (not the package), puts the venv first on `CMAKE_PREFIX_PATH`, passes `COLCON_UV_VENV` / `COLCON_UV_SITE_PACKAGES`, and colcon-ros's ament_cmake tasks build and test the package. `colcon uv install` handles these packages too. Needs colcon-ros. Packages without `build-type` are built as Python packages exactly as before.
- New example `tests/examples/uv_cpp_example`: Eigen from the `cmeel-eigen` wheel.
- README: `uv_ament_cmake_example` is a Python package (its `CMakeLists.txt` is not used), not a C++/Python hybrid.

## 0.4.1

- Executable symlinks are created under `--merge-install` too.
