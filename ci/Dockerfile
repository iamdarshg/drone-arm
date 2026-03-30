# CI Toolchain Image – Drone-Arm Firmware
#
# Provides a fully pinned build environment that matches the GitHub Actions
# ubuntu-latest workflow steps.  Build this image locally to reproduce CI
# results without network access.
#
# Build:
#   docker build -t drone-arm-ci ci/
#
# Run (interactive, repo root mounted):
#   docker run --rm -it -v "$(pwd)":/work -w /work drone-arm-ci bash
#
# Configure and test inside the container:
#   meson setup builddir
#   ninja -C builddir
#   meson test -C builddir --print-errorlogs

FROM ubuntu:22.04

# Avoid interactive prompts during package installation.
ENV DEBIAN_FRONTEND=noninteractive

# Pin tool versions via apt.  Update these to upgrade the toolchain; record
# any version change in a PR so CI history stays reproducible.
#
# Tool versions installed by ubuntu:22.04 apt (as of 2024-01):
#   gcc        11.4
#   clang      14.0
#   clang-tidy 14.0
#   cppcheck   2.7
#   meson      0.61 (upgraded via pip below)
#   ninja      1.10
#   lcov       1.15
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ninja-build \
    python3 \
    python3-pip \
    clang \
    clang-tidy \
    cppcheck \
    lcov \
    gcc \
    git \
    ca-certificates \
  && rm -rf /var/lib/apt/lists/*

# Install a recent Meson via pip for consistent behaviour across distros.
# Pin to a known-good release; update deliberately.
RUN pip3 install --no-cache-dir meson==1.3.2

# Smoke-test tool availability.
RUN meson --version \
  && ninja --version \
  && cppcheck --version \
  && clang-tidy --version \
  && lcov --version

WORKDIR /work
