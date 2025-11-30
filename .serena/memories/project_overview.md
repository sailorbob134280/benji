# Benji Project Overview

## Purpose
Benji is a small autonomous security robot designed for indoor surveillance and monitoring.

## Architecture
The project implements a **dual-processor architecture**:

1. **Benji RT** - Real-time control core running Zephyr RTOS on an MCU (STM32F405)
   - Handles: motor control, sensor acquisition, real-time tasks
   - Target board: Adafruit Feather STM32F405 (development), custom PCB planned
   - Language: C/C++ (C++23 enabled)

2. **Benji Compute** - Compute platform running embedded Linux (Yocto/AutonomOS) on Raspberry Pi 5
   - Handles: image processing, path planning, compute-intensive tasks
   - Base system: AutonomOS (from previous projects)
   - Built via Kas/Yocto

## Hardware Baseline

**Sensors:**
- Camera (image processing)
- 6-axis IMU (accelerometer + gyro)
- 3-axis magnetometer
- Linear ToF distance sensor
- 4x quadrature encoders (wheel odometry)

**Actuators:**
- 4x DC gearmotors with individual drivers
- 1x servo motor (camera tilt)

## Technology Stack

- **Python**: 3.13+ (project language for tooling)
- **Package Manager**: `uv` (fast Python package installer)
- **Build System**: `moon` (monorepo task runner)
- **West**: Zephyr's meta-tool for managing repositories
- **Kas**: Tool for Yocto project setup
- **Just**: Command runner (wrapper around moon tasks)
- **Documentation**: Sphinx with MyST (Markdown), nbsphinx (Jupyter notebooks)
- **Embedded**: Zephyr RTOS, Yocto/BitBake
- **Commit Convention**: Commitizen with conventional commits

## Communication
MCU ↔ SBC communication via high-bandwidth serial (protocol TBD)

## Update Mechanisms
- **MCU**: DFU (Device Firmware Update)
- **Compute**: OTA (likely Mender)

## Development Status
Early stage - baseline hardware/software being established.
