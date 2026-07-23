# Startup Guide

> Last Updated: 2026-07-10

This document describes the complete startup procedure for the Capstone Design environment.

---

# Prerequisites

The following software should already be installed.

## Host System

- Ubuntu 22.04 LTS
- Docker
- ROS2 Humble
- OpenHarmony Robot Toolchain
- QEMU
- Git
- HDC

---

# Directory Structure

Example:

```text
~
├── oh_robot_sim/
├── OpenHarmony/
├── ohos-robot-toolchain/
└── capstone-openharmony-integration/
```

---

# Startup Sequence

The system **must** be started in the following order.

```text
Docker

↓

Host ROS2

↓

OpenHarmony QEMU

↓

HDC

↓

DDS Verification

↓

Robot Services
```

---

# Step 1 - Start Docker

```bash
cd ~/oh_robot_sim
./ros-humble.sh
```

Verify:

```bash
docker ps
```

Expected container:

```text
ros2-dev
```

---

# Step 2 - Enter Docker

Open a new terminal.

```bash
docker exec -it ros2-dev bash
```

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /root/workspace/install/local_setup.bash
```

Verify:

```bash
ros2 topic list
```

Expected topics include:

```text
/tf
/odom
/joint_states
/clock
```

---

# Step 3 - Start OpenHarmony QEMU

Host terminal:

```bash
cd ~/OpenHarmony
sudo ./qemu_run_client.sh
```

Wait until the system finishes booting.

---

# Step 4 - Connect HDC

Host terminal:

```bash
export PATH=$HOME/ohos-robot-toolchain/linux/toolchains:$PATH

hdc tconn 192.168.122.111:55555
```

Verify:

```bash
hdc list targets -v
```

Expected:

```text
Connected
```

---

# Step 5 - Enter OpenHarmony Shell

```bash
hdc shell
```

Inside OpenHarmony:

```bash
cd /data
. ./ros2ohos.env
```

Verify:

```bash
ros2 topic list
```

Expected topics:

```text
/tf
/odom
/joint_states
/clock
```

---

# Step 6 - Verify DDS

Host:

```bash
ros2 topic list
```

OpenHarmony:

```bash
ros2 topic list
```

Both should display the same robot topics.

---

# Step 7 - Test ROS2 Service

Host:

```bash
python3 stage2_arm_server.py
```

OpenHarmony:

```bash
ros2 service call \
/stage2_arm/move \
std_srvs/srv/Trigger "{}"
```

Expected:

Host prints:

```text
RECEIVED COMMAND FROM OH
```

---

# Shutdown

Stop OpenHarmony:

```text
Ctrl + C
```

Stop Docker container:

```bash
exit

docker stop ros2-dev
```

---

# Troubleshooting

## HDC Offline

Verify:

```bash
hdc list targets -v
```

Reconnect:

```bash
hdc tconn 192.168.122.111:55555
```

---

## ROS2 Cannot Discover Topics

Check:

- ROS_DOMAIN_ID
- RMW_IMPLEMENTATION
- CYCLONEDDS_URI

Verify DDS configuration.

---

## Docker Cannot Start

Check:

```bash
docker ps -a
```

Restart:

```bash
cd ~/oh_robot_sim
./ros-humble.sh
```

---

# Daily Checklist

Before development:

- [ ] Docker running
- [ ] ROS2 sourced
- [ ] QEMU running
- [ ] HDC connected
- [ ] DDS working
- [ ] Robot topics visible
- [ ] Service communication verified

---

# Related Documents

- architecture.md
- handover.md
