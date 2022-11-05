# Raven Description

The URDF for the Raven robot, and related files.

The launch file will also launch the joint state publisher and robot state publisher so you can use rviz2 to
visualize the robot without needing other packages. Optionally, you can also launch the [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher_gui) node.

Launch parameters:

- use_sim_time
    
    See [Clock and Time](https://design.ros2.org/articles/clock_and_time.html)

- use_state_pub_gui
    
    Also launch the [joint_state_publisher_gui](http://wiki.ros.org/joint_state_publisher_gui) node

E.g.:

    ros2 launch raven_description raven_description.launch.py -- use_state_pub_gui:=true
