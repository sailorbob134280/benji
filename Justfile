default: help

# Install the pre-commit hooks
bootstrap:
  pre-commit install --hook-type commit-msg --hook-type pre-commit --hook-type pre-push

# Run all setup tasks
setup:
  moon run :setup

build target:
  moon run {{target}}:build

# Build all projects
build-all:
  moon run :build

# Clean all projects
clean:
  moon run :clean

# Print this help message
help:
  just --list
