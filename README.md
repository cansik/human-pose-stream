# Human Pose Stream
A simple human pose estimation software that streams the pose data over OSC.

The software is an adapted example by Intel's inference engine which uses the lightweight-openpose multi-human model.

- Adds possibility to choose camera device
- Adds osc output ([tnyosc](https://github.com/toshiroyamada/tnyosc))

## Build

### Dependencies

Please install following dependencies on your System:

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
./human_pose_stream -m 'human-pose-estimation-0001.xml' -i cam -index 1
```

## About

The project is slightly adapted from the Human Pose example Intel provides.
