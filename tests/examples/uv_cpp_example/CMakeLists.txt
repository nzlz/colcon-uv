cmake_minimum_required(VERSION 3.16)
project(uv_cpp_example CXX)

find_package(ament_cmake REQUIRED)
# colcon-uv put the venv first on CMAKE_PREFIX_PATH, so this is the PyPI Eigen even when a system
# one (libeigen3-dev) is installed.
find_package(Eigen3 3.4 CONFIG REQUIRED)
message(STATUS "uv_cpp_example: Eigen3_DIR=${Eigen3_DIR} (venv ${COLCON_UV_VENV})")

add_executable(uv_cpp_example src/main.cpp)
target_link_libraries(uv_cpp_example Eigen3::Eigen)
target_compile_features(uv_cpp_example PRIVATE cxx_std_17)

install(TARGETS uv_cpp_example DESTINATION lib/${PROJECT_NAME})

ament_package()
