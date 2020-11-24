#!/usr/bin/env bash
source /opt/intel/openvino_2020.1.023/bin/setupvars.sh
echo "running humane pose stream"

./human_pose_stream -m 'human-pose-estimation-0001.xml' -i rs -index 0