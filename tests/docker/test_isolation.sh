#!/bin/bash

# Colors for logging
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${PURPLE}[STEP]${NC} $1"
}

log_separator() {
    echo -e "${CYAN}========================================${NC}"
}

# Error handling
set -e
trap 'log_error "Isolation test failed at line $LINENO"' ERR

main() {
    log_separator
    log_info "Starting Dependency Isolation Test"
    log_separator

    cd /ros2_ws

    # Source ROS and workspace
    log_step "Sourcing ROS and workspace environment"
    source /opt/ros/${ROS_DISTRO}/setup.sh
    source install/setup.sh
    log_success "Environment sourced"

    # Test virtual environment isolation
    log_step "Testing virtual environment isolation"

    # Check Python package venv
    python_venv="/ros2_ws/install/uv_ament_python_example/venv"
    cmake_venv="/ros2_ws/install/uv_ament_cmake_example/venv"

    if [ -d "$python_venv" ]; then
        log_success "Python package venv exists: $python_venv"
    else
        log_error "Python package venv not found: $python_venv"
        exit 1
    fi

    if [ -d "$cmake_venv" ]; then
        log_success "CMake package venv exists: $cmake_venv"
    else
        log_error "CMake package venv not found: $cmake_venv"
        exit 1
    fi

    # Test numpy version isolation
    log_step "Testing numpy version isolation"

    # Get numpy version from Python package venv
    python_numpy_version=$($python_venv/bin/python -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "ERROR")
    cmake_numpy_version=$($cmake_venv/bin/python -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "ERROR")

    log_info "Python package numpy version: $python_numpy_version"
    log_info "CMake package numpy version: $cmake_numpy_version"

    if [ "$python_numpy_version" = "1.26.0" ]; then
        log_success "Python package using correct numpy version (1.26.0)"
    else
        log_error "Python package using wrong numpy version: $python_numpy_version (expected 1.26.0)"
        exit 1
    fi

    if [ "$cmake_numpy_version" = "2.2.0" ]; then
        log_success "CMake package using correct numpy version (2.2.0)"
    else
        log_error "CMake package using wrong numpy version: $cmake_numpy_version (expected 2.2.0)"
        exit 1
    fi

    if [ "$python_numpy_version" != "$cmake_numpy_version" ]; then
        log_success "Dependency isolation confirmed - different numpy versions in each package"
    else
        log_error "Dependency isolation failed - both packages using same numpy version"
        exit 1
    fi

    # Test package isolation
    log_step "Testing package isolation"

    # Check that each package has its own site-packages
    python_site_packages="$python_venv/lib/python3.12/site-packages"
    cmake_site_packages="$cmake_venv/lib/python3.12/site-packages"

    if [ -d "$python_site_packages" ]; then
        log_success "Python package has isolated site-packages"
    else
        log_error "Python package missing site-packages directory"
        exit 1
    fi

    if [ -d "$cmake_site_packages" ]; then
        log_success "CMake package has isolated site-packages"
    else
        log_error "CMake package missing site-packages directory"
        exit 1
    fi

    # Test executable isolation
    log_step "Testing executable isolation"

    # Check that executables exist in the right locations
    python_ping_exec="/ros2_ws/install/uv_ament_python_example/lib/uv_ament_python_example/ping_node"
    cmake_pong_exec="/ros2_ws/install/uv_ament_cmake_example/lib/uv_ament_cmake_example/pong_node"

    if [ -f "$python_ping_exec" ] || [ -L "$python_ping_exec" ]; then
        log_success "ping_node executable found in Python package"
    else
        log_error "ping_node executable not found: $python_ping_exec"
        exit 1
    fi

    if [ -f "$cmake_pong_exec" ] || [ -L "$cmake_pong_exec" ]; then
        log_success "pong_node executable found in CMake package"
    else
        log_error "pong_node executable not found: $cmake_pong_exec"
        exit 1
    fi

    # Test that executables use their respective venvs
    log_step "Testing executable-venv binding"

    # Run executables and check they use correct numpy versions
    timeout 3s $python_ping_exec >/tmp/python_exec_test.log 2>&1 || true
    timeout 3s $cmake_pong_exec >/tmp/cmake_exec_test.log 2>&1 || true

    if grep -q "Ping Node started from uv_ament_python_example venv" /tmp/python_exec_test.log && grep -q "numpy version: 1.26.0" /tmp/python_exec_test.log; then
        log_success "ping_node executable uses Python package's numpy (1.26.0)"
    else
        log_error "ping_node executable not using correct numpy version"
        cat /tmp/python_exec_test.log
        exit 1
    fi

    if grep -q "Pong Node started from uv_ament_cmake_example venv" /tmp/cmake_exec_test.log && grep -q "numpy version: 2.2.0" /tmp/cmake_exec_test.log; then
        log_success "pong_node executable uses CMake package's numpy (2.2.0)"
    else
        log_error "pong_node executable not using correct numpy version"
        cat /tmp/cmake_exec_test.log
        exit 1
    fi

    # Test system package access
    log_step "Testing system package access"

    # Both venvs should have access to system ROS packages
    python_rclpy_test=$($python_venv/bin/python -c "import rclpy; print('OK')" 2>/dev/null || echo "ERROR")
    cmake_rclpy_test=$($cmake_venv/bin/python -c "import rclpy; print('OK')" 2>/dev/null || echo "ERROR")

    if [ "$python_rclpy_test" = "OK" ]; then
        log_success "Python package venv can access system rclpy"
    else
        log_error "Python package venv cannot access system rclpy"
        exit 1
    fi

    if [ "$cmake_rclpy_test" = "OK" ]; then
        log_success "CMake package venv can access system rclpy"
    else
        log_error "CMake package venv cannot access system rclpy"
        exit 1
    fi

    # Test dependency file analysis
    log_step "Analyzing dependency files"

    # Check pyproject.toml files have correct versions
    python_pyproject="/ros2_ws/src/examples/uv_ament_python_example/pyproject.toml"
    cmake_pyproject="/ros2_ws/src/examples/uv_ament_cmake_example/pyproject.toml"

    if grep -q "numpy==1.26.0" "$python_pyproject"; then
        log_success "Python package pyproject.toml specifies numpy 1.26.0"
    else
        log_error "Python package pyproject.toml doesn't specify correct numpy version"
        exit 1
    fi

    if grep -q "numpy==2.2.0" "$cmake_pyproject"; then
        log_success "CMake package pyproject.toml specifies numpy 2.2.0"
    else
        log_error "CMake package pyproject.toml doesn't specify correct numpy version"
        exit 1
    fi

    log_separator
    log_success "Dependency Isolation Test PASSED!"
    log_separator

    log_info "Isolation Test Results Summary:"
    log_info "✓ Virtual environments are isolated"
    log_info "✓ Different numpy versions (1.26.0 vs 2.2.0)"
    log_info "✓ Separate site-packages directories"
    log_info "✓ Executables use correct venv dependencies"
    log_info "✓ System packages (rclpy) accessible from both venvs"
    log_info "✓ Configuration files specify correct versions"

    log_success "Perfect dependency isolation achieved!"
}

main "$@"
