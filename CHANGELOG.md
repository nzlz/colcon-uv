# Changelog

## 0.5.0

- C++ packages with PyPI dependencies: `build-type = "ament_cmake"` in `[tool.colcon-uv-ros]` makes an ament_cmake package (CMakeLists.txt + package.xml) take its PyPI dependencies from its own venv (`install/<package>/venv`). colcon-uv installs the dependencies (not the package), puts the venv first on `CMAKE_PREFIX_PATH`, passes `COLCON_UV_VENV` / `COLCON_UV_SITE_PACKAGES`, and colcon-ros's ament_cmake tasks build and test the package. `colcon uv install` handles these packages too. Needs colcon-ros. Packages without `build-type` are built as Python packages exactly as before.
- New example `tests/examples/uv_cpp_example`: Eigen from the `cmeel-eigen` wheel.
- README: `uv_ament_cmake_example` is a Python package (its `CMakeLists.txt` is not used), not a C++/Python hybrid.

## 0.4.1

- Executable symlinks are created under `--merge-install` too.
