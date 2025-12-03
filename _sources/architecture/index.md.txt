# Architecture

Benji overall operates as a split system with a real time processor and a compute platform. The real time processor handles all low-latency real-time tasks such as:

- Sensor input and processing
- State estimation
- Actuator control
- Short-term waypoint following

The compute platform handles higher-level tasks such as:

- Vision processing
- Visual SLAM
- OTA updates
- Path planning
- User interface and monitoring

## Software

[![Benji Software Architecture](benji_sw_arch.png)](benji_sw_arch.png)

### Real Time Software

The real-time software is built aroudn the Zephyr RTOS, with all components written in C++. There are a few key components that make up the bulk of the software:

- **Extended Kalman Filter**: Responsible for sensor fusion and state estimation, both of the lower-level sensors (IMU, magnetometer, and wheel encoders) as well as higher-level, lower-rate (and possibly jittery) estimates from visual odometry.
- **Controller**: Takes a state estimate and a reference trajectory and producing actuator commands to follow the trajectory as closely as possible.
- **Motor CLCs**: Low level closed-loop controllers for the individual wheel motors, responsible for tracking velocity commands from the higher-level controller.
- **Command and Telemetry Processor**: Accepts commands from the compute platform (Protobuf) and debug shell to control the robot, and sends telemetry back to the compute platform for monitoring and logging.
- **Debug Shell**: A CLI utility for control and calibration via a USB connection.

Zephyr provides a device-driver model that is quite similar to Linux, as well as a few convenience abstractions such as the Sensor API, which makes hardware selection less critical in the early stages of development. It also has significan driver support for the sensors and peripherals used.

### Compute Platform Software

The compute platform is an embedded microservice architecture leveraging a blend of C++ and Go services (with Python for prototyping and some vision tasks). Each service communicates primarily via Protobuf messages on a [NATS](https://nats.io/) fabric. The main services are:

- **Camera Processor**: Captures frames from the camera at a configurable rate, performs any necessary pre-processing (undistortion, color correction, etc), and publishes the frames to the NATS fabric for consumption by other services.
- **Visual SLAM**: Subscribes to camera frames, performs visual SLAM to produce pose estimates and a local map, and publishes that data to the NATS fabric.
- **Path Planner**: Subscribes to pose estimates and map data and generates global and local paths for the robot to follow, publishing the paths to the NATS fabric.
- **RT Gateway**: Similar to the Command and Telemetry Processor on the real-time side, this service handles communication with the real-time processor, sending commands and receiving telemetry.
- **Data Recorder**: Subscribes to various telemetry topics on the NATS fabric and forwards them to InfluxDB for monitoring and analysis.
- **Monitoring**: A simple Grafana/InfluxDB stack for basic visualization of telemetry data.
- **Update Manager**: Handles OTA updates for both the compute platform and the real-time processor. This is accomplished via primarily via Mender, with some custom tooling for the real-time side.
- **API/Web UI**: Provides a REST API and web-based user interface for monitoring and controlling the robot.

## Deep Dives

```{toctree}
---
maxdepth: 1
---
rt_compute_api
```

## Architectural Decision Records

This project leverages Architectural Decision Records (ADRs) to document important decisions made during the design and development of Benji.

```{toctree}
---
maxdepth: 1
glob: true
---
../adr/*
```
