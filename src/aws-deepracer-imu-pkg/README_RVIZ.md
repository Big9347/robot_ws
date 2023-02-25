# Readme instructions for running this fork of the code to visualize imu in RVIZ

1. git clone this repo

2. update the config file [] based on your hardware setup especially device_address of bmi160 connected to your RPi

3. get the code from ros2_imu_tools repo by running this shell script while in root folder of the aws-deepracer-imu-pkg workspace

```
. imu_pkg/scripts/load_ros2_imu_tools_repo.sh

```

This will do two things get the ros2_imu_tools repo imported into the workspace and install any dependencies required to run those packages

4. launch robot nodes and host nodes

ros2 launch imu_pkg imu_rviz_robot.launch.py

ros2 launch imu_pkg imu_rviz_host.launch.py

