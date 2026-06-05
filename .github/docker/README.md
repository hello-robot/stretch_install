# Docker-Based Stretch Installation Testing

This directory contains utility scripts and configurations for building and testing the Stretch Robot Installation script (`stretch_new_robot_install.sh`) in an isolated Docker container. 

A GitHub Actions workflow is also configured to run these tests automatically on push or pull requests to the `master` branch.

---

## 1. Running Tests Locally

You can run the build and verification steps locally on any system with Docker installed.

### Step A: Build the Docker Image
The `build.sh` script handles building the Docker image. It accepts a specific robot Fleet ID to mock the calibration and configuration files.

To build the default image (`stretch-install:stretch-se3-0000`):
```bash
./.github/docker/build.sh
```

To build for a specific model (e.g., `stretch-re2-0001`):
```bash
./.github/docker/build.sh -f stretch-re2-0001
```

To build with a custom image name:
```bash
./.github/docker/build.sh -f stretch-re2-0001 -n custom-stretch-test
```

### Step B: Run the Verification Tests
The `test.sh` script runs a suite of 12 verification checks (e.g., checking configuration files, installed packages, ROS Humble installation, directory structures, and log status) against the built image.

To test the default image:
```bash
./.github/docker/test.sh -i stretch-install:stretch-se3-0000
```

To test a custom-named image:
```bash
./.github/docker/test.sh -i custom-stretch-test:stretch-re2-0001
```

---

## 2. Shared Utilities (`common.sh`)
Both scripts source [common.sh](./common.sh) for shared configurations:
* **Color Definitions & Log Handlers**: Provides unified log output styling (`[INFO]`, `[PASS]`, `[FAIL]`, etc.).
* **Docker Check**: Detects if your user is in the `docker` group and automatically prepends `sudo` to docker commands if root permissions are required.

---

## 3. Automated GitHub Actions Workflow
The workflow file at [test-installation.yaml](../workflows/test-installation.yaml) runs these same scripts automatically in GitHub Actions for every pull request and push to `master`. 

It uses a matrix strategy to test multiple models (e.g., `stretch-se3-0000` and `stretch-re2-0001`) concurrently:
1. Checks out the repository.
2. Sets up Docker Buildx.
3. Invokes `build.sh` to compile the Docker image.
4. Invokes `test.sh` to run the verification tests.
