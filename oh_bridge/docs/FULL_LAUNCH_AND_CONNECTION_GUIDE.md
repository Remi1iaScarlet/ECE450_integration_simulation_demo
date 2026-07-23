# Complete Launch and Connection Guide  
## LLM → OpenHarmony → ROS2 → ROS1 → Piper

This guide documents the complete startup, connection, verification, recovery, and shutdown procedure for the current capstone integration.

---

## 1. System Architecture

```text
User natural-language command
        |
        v
Anthropic LLM
        |
        v
robot-command-demo FastAPI
http://127.0.0.1:8000
        |
        | GET /api/queue/current-posted
        v
OpenHarmony QEMU queue agent
192.168.122.x
        |
        | ROS2 std_msgs/String
        v
/stage2_arm/task_command
        |
        v
stage2_adapter.py
        |
        | ROS2 std_msgs/String
        v
/visual_grasp/command_json
        |
        | TCP port 5005
        v
tcp_ros1_receiver.py
        |
        | ROS1 std_msgs/String
        v
/visual_grasp/command_json
        |
        v
visual_grasp_ros1_gateway
        |
        v
Piper dry-run backend or real Piper over can0
```

The key design requirement is that OpenHarmony actively reads the LLM command and publishes the ROS2 task command.

---

## 2. Important Project Paths

```text
~/capstone_design/capstone-openharmony-integration
~/capstone_design/robot-command-demo
~/capstone_design/visual_grasp
~/OpenHarmony
~/oh_robot_sim
```

Important integration files:

```text
capstone-openharmony-integration/
├── agent/
│   └── oh_queue_agent.py
├── bridge/
│   ├── stage2_adapter.py
│   ├── tcp_ros2_sender.py
│   └── tcp_ros1_receiver.py
├── configs/
│   ├── cyclonedds_host.xml
│   ├── cyclonedds_oh.xml
│   ├── oh_runtime.env
│   └── fastapi-runtime.env.example
├── scripts/
│   ├── install_permanent.sh
│   ├── start_all.sh
│   ├── check_all.sh
│   └── stop_all.sh
├── systemd/
│   └── robot-command-demo.service
└── docker-compose.bridge.yml
```

---

## 3. Network Layout

### Ubuntu host

```text
FastAPI:
127.0.0.1:8000

FastAPI as seen by OpenHarmony:
192.168.122.1:8000

ROS2-to-ROS1 TCP bridge:
127.0.0.1:5005
```

### OpenHarmony QEMU

Typical address:

```text
192.168.122.111
```

Do not permanently assume `.111`. Detect the current address using:

```bash
grep "my address" ~/OpenHarmony/kernel.log | tail -n 1
```

### DDS

Host CycloneDDS is bound to:

```text
192.168.122.1
```

This is the QEMU/libvirt bridge address, not the Wi-Fi address.

Changing Wi-Fi normally does not affect:

```text
OpenHarmony ↔ Ubuntu host
OpenHarmony ↔ ROS2 DDS
ROS2 ↔ ROS1 TCP bridge
Piper can0
```

Changing Wi-Fi may affect Anthropic API access and Clash Verge.

---

# Part A — One-Time Installation

## 4. Prepare the LLM Environment

### 【Host terminal】

```bash
cd ~/capstone_design/robot-command-demo
```

Confirm the private environment file exists:

```bash
test -f .env && echo ".env exists"
```

Confirm Git ignores it:

```bash
git check-ignore -v .env
```

The `.env` file must contain the current valid provider configuration and API key.

Never run:

```bash
cat .env
```

Never commit `.env`.

Search the repository for accidentally exposed keys:

```bash
grep -RIn \
  --exclude-dir=.git \
  --exclude='.env' \
  --exclude='.env.example' \
  -E 'sk-ant-|LLM_API_KEY=sk-' .
```

If a previous key was exposed, revoke it and replace it with a new one.

---

## 5. Install the Permanent Startup Configuration

### 【Host terminal】

```bash
cd ~/capstone_design/capstone-openharmony-integration
chmod +x scripts/*.sh agent/oh_queue_agent.py
./scripts/install_permanent.sh
```

This installs:

```text
FastAPI systemd user service
Persistent runtime environment
Dedicated headless ROS2 bridge configuration
Permanent OpenHarmony queue agent deployment support
```

Check the FastAPI service:

```bash
systemctl --user status robot-command-demo.service --no-pager -l
```

Do not manually run Uvicorn after installing the service.

Wrong:

```bash
uvicorn backend.main:app --host 0.0.0.0 --port 8000
```

Correct:

```bash
systemctl --user start robot-command-demo.service
```

---

## 6. Check the FastAPI Proxy Configuration

Runtime proxy file:

```text
~/.config/robot-command-demo/runtime.env
```

Expected proxy variables:

```bash
ALL_PROXY=
all_proxy=
HTTP_PROXY=http://127.0.0.1:7897
HTTPS_PROXY=http://127.0.0.1:7897
http_proxy=http://127.0.0.1:7897
https_proxy=http://127.0.0.1:7897
NO_PROXY=127.0.0.1,localhost,192.168.122.1,192.168.122.111
no_proxy=127.0.0.1,localhost,192.168.122.1,192.168.122.111
```

Start Clash Verge before using the LLM.

Test Anthropic connectivity:

```bash
curl -I https://api.anthropic.com
```

A response such as HTTP 404 from the root URL still proves that the network connection reached Anthropic.

---

# Part B — Daily Startup

## 7. Recommended Daily Startup

### 【Host terminal】

Start Clash Verge first.

Then run:

```bash
cd ~/capstone_design/capstone-openharmony-integration
./scripts/start_all.sh
```

This command starts:

```text
1. FastAPI systemd user service
2. ROS1 roscore
3. Piper gateway
4. ROS1 TCP receiver
5. Dedicated ROS2 bridge container
6. OpenHarmony QEMU
7. HDC connection
8. OpenHarmony queue agent
```

The script intentionally does not arm a real Piper.

---

## 8. Verify the Whole Stack

### 【Host terminal】

```bash
cd ~/capstone_design/capstone-openharmony-integration
./scripts/check_all.sh
```

A healthy result should show:

```text
FastAPI health: ok
ROS1 roscore: running
Piper gateway: running
tcp_ros1_receiver.py: running
capstone-ros2-bridge: running
/stage2_arm/task_command: visible
/visual_grasp/command_json: visible
HDC: Connected
OH queue agent: started
```

When no real Piper is connected, this is normal:

```text
can0 absent
```

---

# Part C — Component-by-Component Verification

## 9. LLM and FastAPI

### Health check

```bash
curl --noproxy '*' \
  http://127.0.0.1:8000/api/health
```

Expected:

```json
{"status":"ok"}
```

### Swagger

Open:

```text
http://localhost:8000/docs
```

### Queue page

Open:

```text
http://localhost:8000/queue
```

### Submit a command

Use:

```json
{
  "command": "pick the bottle"
}
```

Expected result:

```json
{
  "success": true,
  "item": {
    "intent": "pick",
    "validation_passed": true,
    "service_command": "pick",
    "queue_status": "pending"
  }
}
```

The full item should contain:

```json
{
  "llm_output": {
    "intent": "pick",
    "parameters": {
      "source_label": "bottle"
    }
  },
  "validation": {
    "passed": true,
    "service_name": "/stage2_arm/task_command",
    "service_command": "pick"
  }
}
```

### Approve the item

Use:

```text
POST /api/queue/{item_id}/approve
```

Approve an item only once.

If no other item is currently posted, the item becomes:

```text
posted
```

If another item is already posted, it becomes:

```text
approved
```

---

## 10. FastAPI Port Conflict Recovery

Symptom:

```text
ERROR: [Errno 98] address already in use
```

Check the process listening on port 8000:

```bash
sudo lsof -nP -iTCP:8000 -sTCP:LISTEN
```

If an old manually launched Uvicorn is using the port, terminate only that PID:

```bash
kill <OLD_PID>
```

Then restart the service:

```bash
systemctl --user restart robot-command-demo.service
sleep 2
systemctl --user status robot-command-demo.service --no-pager -l
```

Verify:

```bash
curl --noproxy '*' \
  http://127.0.0.1:8000/api/health
```

Do not use:

```bash
pkill python3
```

because it can terminate unrelated ROS and bridge processes.

---

## 11. OpenHarmony QEMU

### Detect the current OH address

```bash
grep "my address" ~/OpenHarmony/kernel.log | tail -n 1
```

Typical output:

```text
my address is 192.168.122.111
```

### Connect HDC manually

```bash
hdc tconn 192.168.122.111:55555
hdc list targets -v
```

Expected:

```text
192.168.122.111:55555 TCP Connected
```

### Enter the OH shell

```bash
hdc shell
```

### OH runtime environment

The permanent environment is deployed as:

```text
/data/oh_runtime.env
```

Manual loading:

```bash
. /data/oh_runtime.env
```

Expected important variables:

```bash
export PATH=/data/install/bin:/data/out/bin:$PATH
export LD_LIBRARY_PATH=/data/install/lib:/data/out/lib:$LD_LIBRARY_PATH
export AMENT_PREFIX_PATH=/data/install
export PYTHONPATH=/data/install/lib/python3.12/site-packages:/data/out/lib/python3.12/site-packages:$PYTHONPATH
export HOME=/data
export ROS_LOG_DIR=/data/.ros/log
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///data/cyclonedds_oh.xml
export LLM_API_BASE=http://192.168.122.1:8000
```

### Check ROS2 topics from OH

```bash
. /data/oh_runtime.env
ros2 topic list
```

Expected topics include:

```text
/stage2_arm/task_command
/visual_grasp/command_json
```

### Check the OH Agent

```bash
cat /data/oh_queue_agent.pid
tail -n 50 /data/oh_queue_agent.log
```

Expected log:

```text
Agent started: api=http://192.168.122.1:8000
Published: {"schema":"visual_grasp.bridge.v1",...}
ACK_MODE=manual
```

### Restart the OH Agent manually

```bash
PID="$(cat /data/oh_queue_agent.pid 2>/dev/null)"
[ -n "$PID" ] && kill "$PID"

nohup sh -c '
  . /data/oh_runtime.env
  python3 /data/oh_queue_agent.py
' >/data/oh_queue_agent.log 2>&1 &
```

### Duplicate protection

The Agent remembers the last dispatched queue ID in:

```text
/data/oh_queue_agent_state.json
```

Check it:

```bash
cat /data/oh_queue_agent_state.json
```

Do not delete this file unless intentionally repeating the same queue item.

To intentionally re-dispatch the same ID:

```bash
rm -f /data/oh_queue_agent_state.json
```

Then restart the Agent.

---

## 12. ROS2 Bridge

The permanent ROS2 bridge runs in:

```text
capstone-ros2-bridge
```

### Check container status

```bash
docker ps --filter name=capstone-ros2-bridge
```

### Check topics

```bash
docker exec capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
'
```

Expected:

```text
/stage2_arm/task_command
/visual_grasp/command_json
```

### Check the topic type

```bash
docker exec capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
ros2 topic type /stage2_arm/task_command
'
```

Expected:

```text
std_msgs/msg/String
```

### Watch incoming OH commands

```bash
docker exec -it capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
ros2 topic echo /stage2_arm/task_command
'
```

Keep this terminal running while testing.

### Watch adapted visual-grasp commands

```bash
docker exec -it capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
ros2 topic echo /visual_grasp/command_json
'
```

### View ROS2 bridge logs

```bash
docker logs -f capstone-ros2-bridge
```

---

## 13. ROS1 and Piper Gateway

ROS1 containers:

```text
visual-grasp-noetic-roscore-1
visual-grasp-noetic-piper_gateway-1
```

### Check containers

```bash
docker ps --format 'table {{.Names}}\t{{.Status}}' |
grep -E 'visual-grasp-noetic-(roscore|piper_gateway)-1'
```

### Enter Piper gateway

```bash
docker exec -it \
  visual-grasp-noetic-piper_gateway-1 \
  bash
```

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
```

### Check ROS1 nodes

```bash
rosnode list
```

Expected:

```text
/rosout
/visual_grasp_ros1_gateway
/visual_grasp_tcp_receiver
```

### Check the ROS1 command topic

```bash
rostopic info /visual_grasp/command_json
```

Expected publisher:

```text
/visual_grasp_tcp_receiver
```

Expected subscriber:

```text
/visual_grasp_ros1_gateway
```

### Check the status topic

```bash
rostopic info /visual_grasp/status_json
```

Expected publisher:

```text
/visual_grasp_ros1_gateway
```

### Watch gateway status

```bash
rostopic echo /visual_grasp/status_json
```

Keep this terminal running during command testing.

### Check the TCP receiver

From the host:

```bash
docker exec \
  visual-grasp-noetic-piper_gateway-1 \
  pgrep -af tcp_ros1_receiver.py
```

View its log:

```bash
docker exec \
  visual-grasp-noetic-piper_gateway-1 \
  tail -n 50 /tmp/tcp_ros1_receiver.log
```

---

# Part D — Complete Command Test

## 14. Test Natural Language to ROS1

### Step 1 — Submit the LLM command

Open Swagger:

```text
http://localhost:8000/docs
```

Submit:

```json
{
  "command": "pick the bottle"
}
```

Record the returned item ID.

### Step 2 — Approve once

Call:

```text
POST /api/queue/{item_id}/approve
```

### Step 3 — Verify the posted item

```bash
curl --noproxy '*' \
  http://127.0.0.1:8000/api/queue/current-posted
```

Expected essential content:

```json
{
  "item": {
    "id": 9,
    "llm_output": {
      "intent": "pick",
      "parameters": {
        "source_label": "bottle"
      }
    },
    "queue_status": "posted"
  }
}
```

### Step 4 — Verify OH publication

```bash
hdc shell "tail -n 30 /data/oh_queue_agent.log"
```

Expected:

```json
{
  "schema": "visual_grasp.bridge.v1",
  "request_id": "llm_9",
  "task": "pick",
  "source_label": "bottle",
  "policy": "first"
}
```

### Step 5 — Verify ROS2

```bash
docker exec -it capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
ros2 topic echo /stage2_arm/task_command
'
```

### Step 6 — Verify ROS1

```bash
docker exec -it visual-grasp-noetic-piper_gateway-1 bash -lc '
source /opt/ros/noetic/setup.bash
rostopic echo /visual_grasp/command_json
'
```

### Step 7 — Verify Piper gateway result

```bash
docker exec -it visual-grasp-noetic-piper_gateway-1 bash -lc '
source /opt/ros/noetic/setup.bash
rostopic echo /visual_grasp/status_json
'
```

---

# Part E — Queue Completion

## 15. Manual ACK Mode

Current permanent configuration:

```text
ACK_MODE=manual
```

This is the recommended mode for real hardware.

Publishing a ROS2 command does not automatically mean the physical robot completed it.

After checking `/visual_grasp/status_json`, mark the queue item manually.

### Mark executed

```bash
curl --noproxy '*' -X POST \
  http://127.0.0.1:8000/api/queue/<ITEM_ID>/executed
```

### Mark failed

```bash
curl --noproxy '*' -X POST \
  http://127.0.0.1:8000/api/queue/<ITEM_ID>/failed
```

### Confirm current posted queue

```bash
curl --noproxy '*' \
  http://127.0.0.1:8000/api/queue/current-posted
```

Expected when empty:

```json
{"item":null}
```

Marking the posted item completed allows the next approved item to be promoted.

---

## 16. Queue Status Meanings

```text
pending   Waiting for human approval
approved  Approved but waiting behind another posted item
posted    Current command available to OpenHarmony
executed  Completed successfully
failed    Execution failed
rejected  Rejected by user or validation
```

Do not approve the same item twice.

Error:

```text
Cannot approve item with status 'approved'
```

means the item was already approved and is waiting behind another posted item.

Resolve the older posted item first by marking it:

```text
executed
failed
rejected
```

---

# Part F — Perception Requirement

## 17. `/object_point`

A `pick` command requires a live perception point.

ROS1 interface:

```text
Topic: /object_point
Type: geometry_msgs/PointStamped
```

Check:

```bash
docker exec -it visual-grasp-noetic-piper_gateway-1 bash -lc '
source /opt/ros/noetic/setup.bash
rostopic echo /object_point
'
```

Important requirements:

```text
frame_id should match the expected camera frame
header.stamp must be current
point age must be less than 1 second
```

A repeated message with an old timestamp is rejected as stale.

Typical errors:

```text
no /object_point has been received
/object_point is stale
```

These are perception-input errors, not LLM, OH, ROS2, or ROS1 bridge errors.

---

# Part G — Real Piper

## 18. Wi-Fi and Real Piper

Changing Wi-Fi normally does not affect Piper because Piper uses:

```text
can0
```

Check:

```bash
ip -details link show can0
```

Monitor CAN:

```bash
candump can0
```

Do not guess the CAN bitrate. Use the teammate's known working configuration or Piper documentation.

---

## 19. Real-Hardware Safety

The startup script must not automatically:

```text
enable real motion
arm the Piper
release an emergency stop
change the CAN bitrate
send motion without current perception
```

Before real testing:

```text
Emergency stop reachable
Workspace clear
Robot firmly mounted
Low initial speed
No person inside the working area
Live /object_point verified
One operator responsible for emergency stop
```

Check the current Piper mode:

```bash
cd ~/capstone_design/visual_grasp

docker compose config |
grep -nEi 'backend|dry.?run|allow.?motion|can|speed'
```

The previously verified safe mode was:

```text
dry_run=true
allow_motion=false
```

Use the exact variables already defined in the current compose file. Do not invent new environment-variable names.

---

# Part H — Logs and Troubleshooting

## 20. FastAPI Logs

```bash
journalctl --user \
  -u robot-command-demo.service \
  -f
```

Recent logs:

```bash
journalctl --user \
  -u robot-command-demo.service \
  -n 100 \
  --no-pager -l
```

---

## 21. ROS2 Logs

```bash
docker logs -f capstone-ros2-bridge
```

Check running processes:

```bash
docker exec capstone-ros2-bridge \
  pgrep -af 'stage2_adapter|tcp_ros2_sender'
```

---

## 22. ROS1 Logs

```bash
docker compose \
  -f ~/capstone_design/visual_grasp/docker-compose.yml \
  logs --tail=100 piper_gateway
```

TCP receiver:

```bash
docker exec \
  visual-grasp-noetic-piper_gateway-1 \
  tail -n 100 /tmp/tcp_ros1_receiver.log
```

---

## 23. OpenHarmony Logs

```bash
hdc shell "tail -n 100 /data/oh_queue_agent.log"
```

Check process:

```bash
hdc shell "cat /data/oh_queue_agent.pid"
```

Check HDC:

```bash
hdc list targets -v
```

---

## 24. CycloneDDS Warning

Current warning:

```text
NetworkInterfaceAddress: deprecated element
```

This is currently non-fatal.

If topics are visible and commands pass between OH and ROS2, do not change the DDS configuration immediately before a real-hardware test.

---

## 25. HDC Cannot Connect

Detect the latest IP:

```bash
grep "my address" ~/OpenHarmony/kernel.log | tail -n 1
```

Reconnect:

```bash
hdc tconn <OH_IP>:55555
hdc list targets -v
```

If the VM is not running:

```bash
cd ~/OpenHarmony
sudo ./qemu_run_client.sh
```

Keep the QEMU terminal running if started in the foreground.

---

## 26. ROS2 Topic Not Visible on OH

On OH:

```bash
. /data/oh_runtime.env

echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"

ros2 topic list
```

Expected:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///data/cyclonedds_oh.xml
```

On the ROS2 container:

```bash
docker exec capstone-ros2-bridge bash -lc '
source /opt/ros/humble/setup.bash
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"
ros2 topic list
'
```

Both sides must use the same ROS domain.

---

## 27. TCP Receiver Not Connected

Check ROS2 sender logs:

```bash
docker logs capstone-ros2-bridge
```

Check TCP port:

```bash
ss -ltnp | grep ':5005'
```

Check receiver process:

```bash
docker exec \
  visual-grasp-noetic-piper_gateway-1 \
  pgrep -af tcp_ros1_receiver.py
```

Restart the application stack:

```bash
cd ~/capstone_design/capstone-openharmony-integration
./scripts/stop_all.sh
./scripts/start_all.sh
```

---

# Part I — Shutdown

## 28. Normal Shutdown

### 【Host terminal】

```bash
cd ~/capstone_design/capstone-openharmony-integration
./scripts/stop_all.sh
```

This stops:

```text
OH queue agent
ROS2 bridge
ROS1/Piper containers
FastAPI service
```

QEMU is intentionally left running.

Stop QEMU only after saving work:

```bash
sudo pkill -f '[q]emu-system-x86_64'
```

---

# Part J — Minimal Daily Cheat Sheet

## 29. Start

```bash
cd ~/capstone_design/capstone-openharmony-integration
./scripts/start_all.sh
```

## 30. Check

```bash
./scripts/check_all.sh
```

## 31. Open

```text
http://localhost:8000/docs
http://localhost:8000/queue
```

## 32. Submit

```json
{
  "command": "pick the bottle"
}
```

## 33. Approve

```text
POST /api/queue/{item_id}/approve
```

## 34. Observe

```bash
docker logs -f capstone-ros2-bridge
```

```bash
hdc shell "tail -f /data/oh_queue_agent.log"
```

```bash
docker exec -it visual-grasp-noetic-piper_gateway-1 bash -lc '
source /opt/ros/noetic/setup.bash
rostopic echo /visual_grasp/status_json
'
```

## 35. Complete Queue Item

Success:

```bash
curl --noproxy '*' -X POST \
  http://127.0.0.1:8000/api/queue/<ITEM_ID>/executed
```

Failure:

```bash
curl --noproxy '*' -X POST \
  http://127.0.0.1:8000/api/queue/<ITEM_ID>/failed
```

## 36. Stop

```bash
./scripts/stop_all.sh
```

---

# Part K — Git Upload

## 37. Remove Generated Files

```bash
cd ~/capstone_design/capstone-openharmony-integration

find . \
  -type d \
  -name __pycache__ \
  -prune \
  -exec rm -rf {} +
```

## 38. Secret Check

```bash
grep -RIn \
  --exclude-dir=.git \
  --exclude='.env' \
  --exclude='.env.example' \
  -E 'sk-ant-|LLM_API_KEY=sk-' .
```

## 39. Review

```bash
git status --short
git diff
```

## 40. Commit Integration Repository

```bash
git add \
  agent \
  bridge \
  configs \
  scripts \
  systemd \
  docs \
  interfaces \
  docker-compose.bridge.yml \
  README.md \
  .gitignore

git diff --cached

git commit -m \
  "Add persistent OpenHarmony LLM-to-Piper integration"

git push -u origin "$(git branch --show-current)"
```

## 41. Commit LLM Repository

```bash
cd ~/capstone_design/robot-command-demo

git status --short

git add \
  backend/schema.py \
  backend/validator.py \
  backend/llm_mapper.py \
  .gitignore

git diff --cached

git commit -m \
  "Add Piper-aligned pick intent"

git remote -v
git push -u origin "$(git branch --show-current)"
```

Confirm that the LLM repository remote belongs to the correct account or fork before pushing.

---

## Final Verified Flow

```text
"pick the bottle"
        |
        v
intent=pick
source_label=bottle
        |
        v
FastAPI queue item posted
        |
        v
OH queue agent reads item
        |
        v
/stage2_arm/task_command
        |
        v
stage2_adapter.py
        |
        v
/visual_grasp/command_json
        |
        v
TCP port 5005
        |
        v
ROS1 visual_grasp gateway
        |
        v
Piper dry-run or real hardware
```
