# Benji the Rover

[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://sailorbob134280.github.io/benji/)

<!-- intro-start -->
Benji is a small autonomous security robot designed for indoor surveillance and monitoring.

## Architecture

Benji uses a dual-processor architecture:

- **Benji RT** — Real-time control on STM32F405 running Zephyr RTOS (motor control, sensor acquisition)
- **Benji Compute** — Compute platform on Raspberry Pi 5 running embedded Linux via Yocto (image processing, path planning)

Communication between the two processors uses Protocol Buffers over serial.

## Tech Stack

| Component           | Technology                                                                                                 |
| ------------------- | ---------------------------------------------------------------------------------------------------------- |
| Build orchestration | [moon](https://moonrepo.dev/)                                                                              |
| Python tooling      | [uv](https://docs.astral.sh/uv/)                                                                           |
| RT firmware         | [Zephyr Project](https://zephyrproject.org/) + [West](https://docs.zephyrproject.org/latest/develop/west/) |
| Compute platform    | [Yocto](https://www.yoctoproject.org/) + [Kas](https://kas.readthedocs.io/)                                |
| API definitions     | [Protocol Buffers](https://protobuf.dev/) + [Buf](https://buf.build/)                                      |
| Documentation       | [Sphinx](https://www.sphinx-doc.org/) + [MyST](https://myst-parser.readthedocs.io/)                        |

## Getting Started

### Prerequisites

- Docker
- [uv](https://docs.astral.sh/uv/)
- [moon](https://moonrepo.dev/docs/install)
- [Just](https://just.systems/) (optional, wraps moon tasks)

### Setup

```bash
# Install pre-commit hooks and sync dependencies
just bootstrap

# Update west modules (Zephyr dependencies)
just setup
```

### Build

```bash
# Build the RT firmware
just build-rt

# Build documentation
just build-doc

# Generate protobuf code
just proto

# List all available commands
just help
```

## AI Usage

AI tools were used to assist in this project. This includes initial drafts of documenation, code snippets, brainstorming, and notebook cleanup. All AI-generated content was reviewed and edited by a human to ensure accuracy and quality.
<!-- intro-end -->
