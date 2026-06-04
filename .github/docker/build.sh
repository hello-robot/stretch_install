#!/bin/bash
# Build script for Stretch Robot Installation Docker image
# This script can be used to build and test the Docker image locally

set -e

# Default values
FLEET_ID="${FLEET_ID:-stretch-se3-0000}"
IMAGE_NAME="${IMAGE_NAME:-stretch-install}"
DOCKERFILE_PATH=".github/docker/Dockerfile"
BUILD_CONTEXT="."

# Source common utilities
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/common.sh" ]; then
    source "$SCRIPT_DIR/common.sh"
else
    echo "Error: common.sh not found in $SCRIPT_DIR"
    exit 1
fi

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--fleet-id)
            FLEET_ID="$2"
            shift 2
            ;;
        -n|--name)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -f, --fleet-id ID    Set the fleet ID (default: stretch-se3-0000)"
            echo "  -n, --name NAME      Set the image name (default: stretch-install)"
            echo "  -h, --help           Display this help message"
            echo ""
            echo "Environment Variables:"
            echo "  FLEET_ID             Set the fleet ID (overridden by -f flag)"
            echo "  IMAGE_NAME           Set the image name (overridden by -n flag)"
            echo ""
            echo "Examples:"
            echo "  $0"
            echo "  $0 -f stretch-re2-1234"
            echo "  $0 -f stretch-se3-0001 -n my-stretch-install"
            echo "  FLEET_ID=stretch-re2-5678 $0"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

# Validate fleet ID format
if [[ ! $FLEET_ID =~ ^stretch-(re1|re2|se3)-[0-9]{4}$ ]]; then
    print_error "Invalid fleet ID format: $FLEET_ID"
    echo "Fleet ID must match the format: stretch-(re1|re2|se3)-XXXX"
    echo "where XXXX is a 4-digit number"
    exit 1
fi

print_info "Building Stretch Robot Installation Docker image"
print_info "Fleet ID: $FLEET_ID"
print_info "Image name: $IMAGE_NAME"
print_info "Dockerfile: $DOCKERFILE_PATH"
echo ""

# Check if we're in the right directory
if [ ! -f "$DOCKERFILE_PATH" ]; then
    print_error "Dockerfile not found at $DOCKERFILE_PATH"
    print_error "Please run this script from the repository root directory"
    exit 1
fi

# Determine if we need to use sudo for docker
detect_docker

# Build the Docker image
print_info "Starting Docker build..."
if $DOCKER_CMD build \
    --build-arg SETUP_FLEET_ID="$FLEET_ID" \
    -t "$IMAGE_NAME:$FLEET_ID" \
    -t "$IMAGE_NAME:latest" \
    -f "$DOCKERFILE_PATH" \
    "$BUILD_CONTEXT"; then
    
    print_info "Docker build completed successfully!"
    echo ""
    print_info "Image tags:"
    echo "  - $IMAGE_NAME:$FLEET_ID"
    echo "  - $IMAGE_NAME:latest"
    echo ""
    print_info "To run the container:"
    echo "  docker run -it --rm $IMAGE_NAME:latest"
    echo ""
    print_info "To verify the installation:"
    echo "  docker run --rm $IMAGE_NAME:latest cat /etc/hello-robot/hello-robot.conf"
    echo ""
else
    print_error "Docker build failed!"
    exit 1
fi

