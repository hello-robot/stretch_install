#!/bin/bash
# Test script for Stretch Robot Installation Docker image
# This script verifies that the Docker installation completed successfully

# Default values
IMAGE_NAME="${IMAGE_NAME:-stretch-install:latest}"

# Source common utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/common.sh" ]; then
    source "$SCRIPT_DIR/common.sh"
else
    echo "Error: common.sh not found in $SCRIPT_DIR"
    exit 1
fi

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    print_info "Running test: $test_name"
    
    if eval "$test_command"; then
        print_success "$test_name"
        ((TESTS_PASSED++))
        return 0
    else
        print_failure "$test_name"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -i, --image NAME     Set the image name to test (default: stretch-install:latest)"
            echo "  -h, --help           Display this help message"
            echo ""
            echo "Examples:"
            echo "  $0"
            echo "  $0 -i stretch-install:stretch-se3-0000"
            exit 0
            ;;
        *)
            print_failure "Unknown option: $1"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

# Detect docker command
detect_docker

print_info "Testing Stretch Robot Installation Docker image: $IMAGE_NAME"
echo ""

# Test 1: Check if image exists
run_test "Image exists" \
    "$DOCKER_CMD image inspect $IMAGE_NAME > /dev/null 2>&1"

# Test 2: Check if /etc/hello-robot/hello-robot.conf exists
run_test "Configuration file exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -f /etc/hello-robot/hello-robot.conf"

# Test 3: Check if HELLO_FLEET_ID is set in configuration
run_test "FLEET_ID configured in /etc/hello-robot/hello-robot.conf" \
    "$DOCKER_CMD run --rm $IMAGE_NAME grep -q 'HELLO_FLEET_ID=' /etc/hello-robot/hello-robot.conf"

# Test 4: Check if stretch_user directory exists
run_test "stretch_user directory exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -d /home/hello-robot/stretch_user"

# Test 5: Check if fleet-specific directory exists in stretch_user
run_test "Fleet-specific directory exists in stretch_user" \
    "$DOCKER_CMD run --rm $IMAGE_NAME bash -c 'FLEET_ID=\$(grep HELLO_FLEET_ID /etc/hello-robot/hello-robot.conf | cut -d= -f2) && test -d /home/hello-robot/stretch_user/\$FLEET_ID'"

# Test 6: Check if log directory exists
run_test "Log directory exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -d /home/hello-robot/stretch_user/log"

# Test 7: Check if installation log was created
run_test "Installation log exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -f /home/hello-robot/stretch_user/log/docker_install.log"

# Test 8: Check if Python is installed
run_test "Python3 is installed" \
    "$DOCKER_CMD run --rm $IMAGE_NAME which python3"

# Test 9: Check if pip is installed
run_test "pip3 is installed" \
    "$DOCKER_CMD run --rm $IMAGE_NAME which pip3"

# Test 10: Check if git is installed
run_test "git is installed" \
    "$DOCKER_CMD run --rm $IMAGE_NAME which git"

# Test 11: Check if .bashrc was updated
run_test ".bashrc contains HELLO_FLEET_ID" \
    "$DOCKER_CMD run --rm $IMAGE_NAME grep -q 'HELLO_FLEET_ID' /home/hello-robot/.bashrc"

# Test 12: Check if .local/bin directory exists
run_test ".local/bin directory exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -d /home/hello-robot/.local/bin"

echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
echo -e "${GREEN}Passed:${NC} $TESTS_PASSED"
echo -e "${RED}Failed:${NC} $TESTS_FAILED"
echo "Total:  $((TESTS_PASSED + TESTS_FAILED))"
echo "========================================"

if [ $TESTS_FAILED -eq 0 ]; then
    print_success "All tests passed!"
    exit 0
else
    print_failure "Some tests failed!"
    exit 1
fi

