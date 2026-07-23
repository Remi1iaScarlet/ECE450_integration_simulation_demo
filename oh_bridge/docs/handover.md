# Project Handover

> Last Updated: 2026-07-10  
> Maintainer: Tim Yu

---

# Project Goal

The objective of this project is to demonstrate that **OpenHarmony OS actively participates in robot control**, instead of having all control logic executed only on the host computer.

Target architecture:

```text
OpenHarmony (QEMU)
        │
        ▼
    ROS2 Humble
        │
        ▼
 Integration Bridge
      /           \
     /             \
 MuJoCo         Piper Robot
Simulation     (ROS Noetic)
```

---

# Current Progress

## ✅ Completed

- [x] Docker development environment
- [x] OpenHarmony EDU (QEMU)
- [x] HDC connection
- [x] ROS2 Humble on OpenHarmony
- [x] CycloneDDS configuration
- [x] DDS communication between OpenHarmony and Host
- [x] OpenHarmony can discover host ROS2 topics
- [x] OpenHarmony successfully calls Host ROS2 service

Verified communication:

```text
OpenHarmony
      │
      ▼
ROS2 Service Call
      │
      ▼
Host Docker
      │
      ▼
Service Callback Triggered
```

---

# Key Configuration

## Host

| Item | Value |
|------|-------|
| ROS Distribution | Humble |
| ROS_DOMAIN_ID | 0 |
| RMW | rmw_cyclonedds_cpp |
| DDS Config | /root/cyclonedds.xml |
| DDS Interface | 192.168.122.1 |

---

## OpenHarmony

| Item | Value |
|------|-------|
| QEMU IP | 192.168.122.111 |
| ROS_DOMAIN_ID | 0 |
| RMW | rmw_cyclonedds_cpp |
| DDS Config | /data/cyclonedds.xml |

---

# Verified Topics

The following ROS2 topics are visible from OpenHarmony:

```text
/tf
/tf_static
/odom
/joint_states
/clock
/camera/*
/scan
/cmd_vel
...
```

---

# Current System Architecture

```text
                   OpenHarmony (QEMU)
                          │
                    ROS2 Service
                          │
                          ▼
                Integration Bridge
                 /               \
                /                 \
         MuJoCo Simulation      ROS1 Piper
```

---

# Team Responsibilities

| Module | Owner | Status |
|---------|-------|--------|
| OpenHarmony Integration | Tim | ✅ |
| MuJoCo Simulation | TBD | 🚧 |
| Piper Integration | TBD | 🚧 |
| LLM Integration | TBD | 📋 |

---

# Current Blockers

## Piper

Current status:

- ROS Noetic
- Robot can move successfully

Need from Piper teammate:

- ROS1 Topic / Service / Action
- Interface name
- Message type
- Example command
- Deployment architecture

---

## MuJoCo

Need from MuJoCo teammate:

- Pick & Place interface
- IK implementation status
- Motion planning interface
- Input message format
- Output status

---

# LLM Deployment Plan

Recommended deployment:

```text
OpenHarmony
      │
      ▼
ROS2 Command
      │
      ▼
Host LLM
      │
      ▼
Robot Command
      │
      ▼
Bridge
      │
      ├── MuJoCo
      └── Piper
```

The LLM **should not run inside OpenHarmony QEMU**.

---

# Next Steps

## High Priority

- [ ] Obtain Piper ROS1 interface
- [ ] Obtain MuJoCo control interface
- [ ] Define unified robot command format
- [ ] Implement Bridge

## Medium Priority

- [ ] Integrate LLM
- [ ] Connect Bridge to MuJoCo
- [ ] Connect Bridge to Piper

## Final Demo

- [ ] OpenHarmony starts successfully
- [ ] HDC connected
- [ ] ROS2 communication verified
- [ ] OpenHarmony sends command
- [ ] Host receives command
- [ ] MuJoCo or Piper executes motion
- [ ] Record demonstration video

---

# Useful Commands

## Connect OpenHarmony

```bash
hdc tconn 192.168.122.111:55555
hdc shell
```

## Start Docker

```bash
cd ~/oh_robot_sim
./ros-humble.sh
```

## Source ROS2

```bash
source /opt/ros/humble/setup.bash
source /root/workspace/install/local_setup.bash
```

## Test Service

```bash
ros2 service call /stage2_arm/move std_srvs/srv/Trigger "{}"
```

---

# Notes

This repository is intended to serve as the **integration repository** for the Capstone Design project.

It is **not** a fork or replacement of `oh_robot_sim`.

Large binary files (QEMU images, SDKs, Docker images, build directories, etc.) should **not** be committed to this repository.
