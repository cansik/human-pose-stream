# Human Pose Stream
A simple human pose estimation software that streams the pose data over OSC.

The software is an adapted example by Intel's inference engine which uses the lightweight-openpose multi-human model.

- Adds possibility to choose camera device
- Adds osc output ([tnyosc](https://github.com/toshiroyamada/tnyosc))

## Simple Setup

A very simple way to test it out is to run the following commands:

```bash
# build and download models
./setup.sh

# let it run
./run.sh
```

## Build

### Dependencies

Please install following dependencies on your System (no Windows Support!):

- [Intel OpenVINO](https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_macos.html)
- [Boost](https://formulae.brew.sh/formula/boost)

And make sure that you are in a shell that has the OpenVINO variables loaded:

```bash
source /opt/intel/openvino/bin/setupvars.sh
```

### Make

To build the solution, just run the build script for your platform:

```
./build.sh
```

The final binary should be avaiable here:

```
~/omz_demos_build/intel64/Release/human_pose_stream
```


## Run

To run the binary, place the model and weights into the same folder and run following command (with OpenVINO variables loaded).

```bash
./human_pose_stream -m 'human-pose-estimation-0001.xml' -i cam -index 0
```

## OSC Output
The OSC data is sent as a bundled message. The first message is the **header**, which tells how many more messages are following in the bundle (necessary for non-bundle OSC implementations). The following messages in the bundle are the different poses and their keypoints.

```
# header
/poses poseCount:int

# poses
/pose id:int score:float x1:float y1:float x2:float y2:float ...
```

Check out the [Processing example](https://github.com/cansik/human-pose-stream/tree/master/examples/HumanPoseReceiver) provided on how to read the data.

## About

The project is slightly adapted from the Human Pose example Intel provides.
