# Raven Bluetooth Joystick Package

This package connects to a Bluetooth Joystick and publishes cmdvel topics.

There are rules files to be placed in /etc/udev/rules located in the udev_rules directory
which will map the joystick (that I use) to a device in the /dev directory.
You will probably need similar udev rules for your joystick device.

Launch is via:

    ros2 launch raven_bluetooth_joystick raven_bluetooth_joystick.launch.py

This node will echo a connection message repeatedly until the joystick node is recognized and connected.

There is a configuration file on the config directory (bluetooth_joystick_yaml) which defines
the  joystick device path to be used (see the udev rules), the topic to be published,
how often to publish cmdvel messages, scaling factors to bring the x and z values within useful range, and the percent of range to be considered a dead zone.