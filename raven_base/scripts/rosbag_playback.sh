#! /usr/bin/bash
ros2 launch puck_description puck_description.launch.py&
ros2 bag play --qos-profile-overrides-path  /home/ros/raven_ws/install/raven_base/share/raven_base/scripts/qos.yaml ~/bag --topics \
/clicked_point \
/odom \
/scan \
/sonar0Sensor \
/sonar1Sensor \
/sonar2Sensor \
/sonar3Sensor \
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
