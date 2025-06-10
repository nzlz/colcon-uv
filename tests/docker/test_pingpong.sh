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
trap 'log_error "Ping-pong test failed at line $LINENO"' ERR

main() {
    log_separator
    log_info "Starting Ping-Pong Demo Test"
    log_separator

    cd /ros2_ws

    # Source ROS and workspace
    log_step "Sourcing ROS and workspace environment"
    source /opt/ros/${ROS_DISTRO}/setup.sh
    source install/setup.sh
    log_success "Environment sourced"

    # Verify packages are available
    log_step "Verifying packages are discoverable"
    if ros2 pkg list | grep -q "uv_ament_python_example"; then
        log_success "uv_ament_python_example package found"
    else
        log_error "uv_ament_python_example package not found"
        exit 1
    fi

    if ros2 pkg list | grep -q "uv_ament_cmake_example"; then
        log_success "uv_ament_cmake_example package found"
    else
        log_error "uv_ament_cmake_example package not found"
        exit 1
    fi

    # Verify executables are available
    log_step "Verifying executables are discoverable"
    if ros2 pkg executables uv_ament_python_example | grep -q "ping_node"; then
        log_success "ping_node executable found"
    else
        log_error "ping_node executable not found"
        exit 1
    fi

    if ros2 pkg executables uv_ament_cmake_example | grep -q "pong_node"; then
        log_success "pong_node executable found"
    else
        log_error "pong_node executable not found"
        exit 1
    fi

    # Test individual nodes first
    log_step "Testing individual nodes"

    # Test ping node (run for 5 seconds)
    log_info "Testing ping_node..."
    timeout 5s ros2 run uv_ament_python_example ping_node >/tmp/ping_output.log 2>&1 || true

    if grep -q "numpy version: 1.26.0" /tmp/ping_output.log; then
        log_success "ping_node using correct numpy version (1.26.0)"
    else
        log_error "ping_node not using expected numpy version"
        cat /tmp/ping_output.log
        exit 1
    fi

    # Test pong node (run for 3 seconds - it just needs to start and show version)
    log_info "Testing pong_node..."
    timeout 3s ros2 run uv_ament_cmake_example pong_node >/tmp/pong_output.log 2>&1 || true

    # Check if it started and showed the correct version
    if grep -q "Pong Node started from uv_ament_cmake_example venv" /tmp/pong_output.log && grep -q "numpy version: 2.2.0" /tmp/pong_output.log; then
        log_success "pong_node using correct numpy version (2.2.0)"
    else
        log_error "pong_node not using expected numpy version"
        log_error "Expected to find both 'Pong Node started from uv_ament_cmake_example venv' and 'numpy version: 2.2.0'"
        exit 1
    fi

    # Test full ping-pong communication
    log_step "Testing ping-pong communication"

    # Start the launch file in background and capture output
    log_info "Starting ping-pong demo..."
    timeout 15s ros2 launch uv_ament_python_example ping_pong_demo.launch.py >/tmp/pingpong_output.log 2>&1 || true

    # Analyze the output
    log_info "Analyzing ping-pong communication..."

    # Check for successful startup
    if grep -q "Ping Node started from uv_ament_python_example venv" /tmp/pingpong_output.log; then
        log_success "Ping node started successfully"
    else
        log_error "Ping node failed to start"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    if grep -q "Pong Node started from uv_ament_cmake_example venv" /tmp/pingpong_output.log; then
        log_success "Pong node started successfully"
    else
        log_error "Pong node failed to start"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    # Check for version isolation
    if grep -q "numpy version: 1.26.0" /tmp/pingpong_output.log && grep -q "numpy version: 2.2.0" /tmp/pingpong_output.log; then
        log_success "Dependency isolation working - different numpy versions detected"
    else
        log_error "Dependency isolation failed - expected different numpy versions"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    # Check for communication
    if grep -q "Published: PING" /tmp/pingpong_output.log; then
        log_success "Ping messages being published"
    else
        log_error "No ping messages found"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    if grep -q "Received: PING" /tmp/pingpong_output.log; then
        log_success "Ping messages being received"
    else
        log_error "No ping message reception found"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    if grep -q "Responded: PONG" /tmp/pingpong_output.log; then
        log_success "Pong responses being sent"
    else
        log_error "No pong responses found"
        cat /tmp/pingpong_output.log
        exit 1
    fi

    # Count message exchanges
    ping_count=$(grep -c "Published: PING" /tmp/pingpong_output.log || echo "0")
    pong_count=$(grep -c "Responded: PONG" /tmp/pingpong_output.log || echo "0")

    log_info "Message exchange summary:"
    log_info "  - Ping messages sent: $ping_count"
    log_info "  - Pong responses sent: $pong_count"

    if [ "$ping_count" -gt 0 ] && [ "$pong_count" -gt 0 ]; then
        log_success "Bidirectional communication working"
    else
        log_error "Communication failed"
        exit 1
    fi

    log_separator
    log_success "Ping-Pong Demo Test PASSED!"
    log_separator

    log_info "Test Results Summary:"
    log_info "✓ Package discovery working"
    log_info "✓ Executable discovery working"
    log_info "✓ Dependency isolation working (numpy 1.26.0 vs 2.2.0)"
    log_info "✓ ROS 2 communication working"
    log_info "✓ Launch files working"
    log_info "✓ Virtual environments isolated"

    log_success "All ping-pong tests passed successfully!"
}

main "$@"
