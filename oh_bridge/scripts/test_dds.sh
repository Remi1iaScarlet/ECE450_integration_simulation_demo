#!/usr/bin/env bash

set -Eeuo pipefail

TOPIC_NAME="/test_dds"

echo "================================="
echo " OpenHarmony ROS2 DDS Test"
echo "================================="
echo

echo "[1/3] Checking ROS2 environment..."

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found"
    echo "Please source ROS2 environment first"
    exit 1
fi


echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not set}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-not set}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-not set}"

echo
echo "[2/3] Available ROS2 nodes:"
ros2 node list || true

echo
echo "[3/3] Publishing test message"
echo "Topic: ${TOPIC_NAME}"
echo

echo "Start receiver on OpenHarmony:"
echo
echo "ros2 topic echo ${TOPIC_NAME}"
echo

ros2 topic pub \
    ${TOPIC_NAME} \
    std_msgs/msg/String \
    "{data: 'DDS communication OK'}" \
    -r 1
