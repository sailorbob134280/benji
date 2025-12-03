default: help

# Print this help message
help:
  just --list

# --- Setup ---

# Install pre-commit hooks and sync Python dependencies
bootstrap:
  pre-commit install --hook-type commit-msg --hook-type pre-commit --hook-type pre-push
  uv sync --locked --all-extras --dev

# Update west modules (Zephyr dependencies)
setup:
  moon run benji:setup

# Build all Docker images
docker-build:
  moon run benji:docker-build

# --- Build ---

# Build a specific target (e.g., just build benji-rt)
build target:
  moon run {{target}}:build

# Build the RT firmware
build-rt:
  moon run benji-rt:build

# Build documentation (HTML + PDF)
build-doc:
  moon run doc:build

# Generate protobuf code
proto:
  moon run proto:generate

# Lint protobuf files
proto-lint:
  moon run proto:lint

# Format protobuf files
proto-fmt:
  moon run proto:format

# --- Clean ---

# Clean a specific target
clean target:
  moon run {{target}}:clean

# Clean all build artifacts
clean-all:
  moon run benji-rt:clean
  moon run doc:clean
  moon run proto:clean
