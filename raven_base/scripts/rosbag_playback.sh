#! /usr/bin/bash
ros2 launch raven_description raven_description.launch.py&
ros2 bag play --qos-profile-overrides-path  /home/ros/raven_ws/install/raven_base/share/raven_base/scripts/qos_playback.yaml ~/bag --topics \
/clicked_point \
/global_costmap/costmap \
/initialpose \
/joint_states \
/local_costmap/costmap \
/map \
/odom \
/scan \
/sonar0Sensor \
/sonar1Sensor \
/sonar2Sensor \
/sonar3Sensor \
/t265/fisheye1/camera_info \
/t265/fisheye1/image_raw \
/tf \
/tf_static \
/tof0Sensor \
/tof1Sensor \
/tof2Sensor \
/tof3Sensor \
/tof4Sensor \
/tof5Sensor \
/tof6Sensor \
/tof7Sensor \
/tofSensor \
