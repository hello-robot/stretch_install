#!/bin/bash
# Common utilities for Stretch Robot Docker build and test scripts

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_failure() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Unified Docker command/privilege detection
DOCKER_CMD=""
detect_docker() {
    if [ -n "$DOCKER_CMD" ]; then
        return 0
    fi
    
    DOCKER_CMD="docker"
    if ! docker ps >/dev/null 2>&1; then
        # Try with sudo
        print_warning "Docker requires elevated privileges. Checking with sudo..."
        if sudo -n docker ps >/dev/null 2>&1; then
            DOCKER_CMD="sudo docker"
            print_warning "Using sudo for Docker commands (user not in docker group)"
        else
            # Need password for sudo
            print_warning "Please enter your password for sudo access to Docker"
            if sudo docker ps >/dev/null 2>&1; then
                DOCKER_CMD="sudo docker"
                print_warning "Using sudo for Docker commands (user not in docker group)"
            else
                print_error "Docker is not running or not accessible"
                exit 1
            fi
        fi
    fi
}
