#!/usr/bin/env bash
source /opt/intel/openvino/bin/setupvars.sh
echo "running humane pose stream"

./human_pose_stream -m 'human-pose-estimation-0001.xml' -i cam -index 0