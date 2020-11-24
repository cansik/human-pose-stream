#!/usr/bin/env bash
echo "running dev environment"

echo "setting up variables..."
source /opt/intel/openvino_2020.1.023/bin/setupvars.sh

# build
sh ./build_debug.sh

# copy posestream
cp ~/omz_demos_build/intel64/Debug/human_pose_stream human_pose_stream

./human_pose_stream -m 'human-pose-estimation-0001.xml' -i rs -index 0