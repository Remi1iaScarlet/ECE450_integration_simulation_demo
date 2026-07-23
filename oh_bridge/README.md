# Capstone OpenHarmony Integration

## Project Overview

This repository contains the integration work for our Capstone Design project.

The goal is to demonstrate that OpenHarmony actively participates in robot control instead of running everything on the host PC.

Current architecture:

```
OpenHarmony (QEMU)
        │
        ▼
    ROS2 Humble
        │
        ▼
 Integration Bridge
      /        \
     /          \
MuJoCo       Piper Robot
Simulation   (ROS Noetic)
```

---

## Current Progress

### Completed

- Docker development environment
- OpenHarmony QEMU
- HDC connection
- ROS2 communication
- CycloneDDS configuration
- OpenHarmony → Host ROS2 Service communication

### In Progress

- ROS1 Bridge
- Piper integration
- LLM integration

---

## Repository Structure

```
docs/
        Documentation

configs/
        DDS configuration

scripts/
        Startup scripts

bridge/
        OpenHarmony bridge

examples/
        Example commands
```

---

## Team

| Module | Owner |
|---------|-------|
| OpenHarmony | Tim |
| MuJoCo Simulation | TBD |
| Piper Robot | TBD |
## Quick Start

### Start Docker

```bash
./scripts/start_host.sh
```

### Start OpenHarmony

```bash
./scripts/start_qemu.sh
```

### Connect HDC

```bash
./scripts/connect_hdc.sh
```

### Test ROS2 Service

```bash
./scripts/test_stage2_arm.sh
```
