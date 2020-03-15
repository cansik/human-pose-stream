# Human Pose Stream
A simple human pose estimation software that streams the pose data over OSC.

The software is an adapted example by Intel's inference engine which uses the lightweight-openpose multi-human model.

- Adds possibility to choose camera device
- Adds osc output ([tnyosc](https://github.com/toshiroyamada/tnyosc))

## Install
- [Intel OpenVINO](https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_macos.html)
- [Boost](https://formulae.brew.sh/formula/boost)

## Run

```bash
./human_pose_stream -m 'human-pose-estimation-0001.xml' -i cam -index 1
```