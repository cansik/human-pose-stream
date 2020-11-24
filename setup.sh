#!/usr/bin/env bash
echo "setting up your environment!"
source /opt/intel/openvino_2020.1.023/bin/setupvars.sh

# build
sh ./build.sh

# copy posestream
cp ~/omz_demos_build/intel64/Release/human_pose_stream human_pose_stream

# download model and config
wget -nc https://github.com/cansik/human-pose-stream/releases/download/0.1.0/human-pose-estimation-0001.bin
wget -nc https://github.com/cansik/human-pose-stream/releases/download/0.1.0/human-pose-estimation-0001.xml

echo "done!"