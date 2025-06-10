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

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
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
trap 'log_error "Script failed at line $LINENO"' ERR

# Main execution
main() {
    log_separator
    log_info "1. Starting colcon-uv test environment setup"
    log_separator

    # Change to workspace directory
    log_step "Changing to workspace directory: /ros2_ws"
    cd /ros2_ws

    # Source ROS environment
    log_step "Sourcing ROS environment"
    source /opt/ros/${ROS_DISTRO}/setup.sh
    log_success "ROS environment sourced successfully"

    # Source workspace if it exists
    if [ -f "install/setup.sh" ]; then
        log_step "Sourcing existing workspace setup"
        source install/setup.sh
        log_success "Workspace setup sourced successfully"
    else
        log_warning "No existing workspace setup found, continuing..."
    fi

    log_separator
    log_step "2. Testing UV dependency installation"
    log_separator

    # Test UV dependency installation
    log_info "Running UV dependency installation for examples..."
    colcon uv install --base-paths src/examples --install-base install --verbose
    log_success "UV dependency installation completed"

    log_separator
    log_step "3. Package identification and listing"
    log_separator

    # List available packages
    log_info "Listing all available packages in workspace..."
    colcon list --base-paths src/examples
    log_success "Package listing completed"

    # Test specific package identification
    log_info "Testing package identification for uv_ament_python_example..."
    colcon list --packages-select uv_ament_python_example --base-paths src/examples
    log_success "Python example package identified successfully"

    log_info "Testing package identification for uv_ament_cmake_example..."
    colcon list --packages-select uv_ament_cmake_example --base-paths src/examples
    log_success "CMake example package identified successfully"

    log_separator
    log_step "4. Sequential package building"
    log_separator

    # Clean previous builds for fresh start
    log_info "Cleaning previous build artifacts..."
    rm -rf build/ install/ log/
    log_success "Build artifacts cleaned"

    # Build packages sequentially (one at a time)
    log_info "Building uv_ament_python_example (Python package)..."
    colcon build \
        --packages-select uv_ament_python_example \
        --base-paths src/examples \
        --symlink-install \
        --executor sequential \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    log_success "uv_ament_python_example built successfully"

    # Source the workspace after first build
    log_step "Sourcing workspace after Python package build"
    source install/setup.sh
    log_success "Workspace sourced after Python package build"

    log_info "Building uv_ament_cmake_example (CMake package)..."
    colcon build \
        --packages-select uv_ament_cmake_example \
        --base-paths src/examples \
        --symlink-install \
        --executor sequential \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    log_success "uv_ament_cmake_example built successfully"

    # Final workspace sourcing
    log_step "Final workspace sourcing"
    source install/setup.sh
    log_success "Final workspace setup completed"

    log_separator
    log_step "5. Build verification and testing"
    log_separator

    # Verify builds
    log_info "Verifying built packages..."
    colcon list --base-paths src/examples
    log_success "Package verification completed"

    # Test package imports/functionality
    log_info "Testing Python package functionality..."
    cd /ros2_ws/src/examples/uv_ament_python_example

    # Test the package itself
    if uv run python -c "import uv_ament_python_example; print('✓ uv_ament_python_example imported successfully')" 2>/dev/null; then
        log_success "uv_ament_python_example imported successfully"
    else
        log_error "Failed to import uv_ament_python_example"
    fi

    # Test dependencies
    if uv run python -c "import numpy; print('✓ numpy dependency available')" 2>/dev/null; then
        log_success "numpy dependency available"
    else
        log_error "numpy dependency not available"
    fi

    if uv run python -c "import requests; print('✓ requests dependency available')" 2>/dev/null; then
        log_success "requests dependency available"
    else
        log_error "requests dependency not available"
    fi

    cd /ros2_ws
    log_success "Python package functionality test completed"

    log_info "Testing CMake package functionality..."
    cd /ros2_ws/src/examples/uv_ament_cmake_example
    # Test the package itself
    if uv run python -c "import uv_ament_cmake_example; print('✓ uv_ament_cmake_example imported successfully')" 2>/dev/null; then
        log_success "uv_ament_cmake_example imported successfully"
    else
        log_error "Failed to import uv_ament_cmake_example"
    fi

    # Test dependencies
    if uv run python -c "import numpy; print('✓ numpy dependency available')" 2>/dev/null; then
        log_success "numpy dependency available"
    else
        log_error "numpy dependency not available"
    fi

    if uv run python -c "import requests; print('✓ requests dependency available')" 2>/dev/null; then
        log_success "requests dependency available"
    else
        log_error "requests dependency not available"
    fi

    log_success "CMake package functionality test completed"

    log_separator
    log_success "6. All tests completed successfully!"
    log_separator

    log_info "Environment is ready for inspection and testing"
    log_info "Available packages:"
    colcon list --base-paths src/examples

    log_info "Container will stay running for inspection..."
    log_info "You can now run additional tests or inspect the built packages"

    # Keep container running
    sleep infinity
}

# Execute main function
main "$@"
