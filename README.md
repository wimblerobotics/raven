# Raven
Raven robot

This is a two-wheeled, differential drive robot. I has an 8-core AMD processor as the main processor, and a custom board which has a Teensy 4.1 at its core which manages most of the sensors and provides safety services, such as trying to prevent the robot from bumping into objects which might be missed by the ROS software.

ROS 2 is used by the main processor, and the Teensy communicates with it using Micro ROS.

Sensors include:
* 8 time-of-flight sensors. A pair is mounted at each core of the robot, near the bottom of the frame. At each corner, the sensors in the pair are mounted at right angles to each other. For instance, at the front, left corner of the robot, one sensor faces forward and the other to the left.

* 4 SONAR sensors. These are mounted at the center point of each of the four sides of the robot, roughly about halfway up the box the forms the robot body.

* 2 redundant motor current sensors. The motors are controlled by a RoboClaw controller which has motor current sensors, but a redundant pair of current sensors provides additional sensing to help prevent melting the motor windings if the motors stall, such as by running into something.

* 2 motor temperature sensors. 

The custom board handles.
* The 8 time-of-flight sensors. These sensors are also averaged to reduce noise.
* The 4 SONAR sensors. These sensors are also averaged to reduce noise.
* The two motor current sensors. If the motors seem to be drawing excessive current, the motor controller will be signaled with an emergency stop to attempt to prevent meltdown of the motor windings.
* The two motor temperature sensors. If the motors become too hot, especially from long term running, the motor controller will be signaled with an emergency stop to attempt to prevent meltdown of the motor windings.
* An 8-channel relay bank which can control power to varioius subsystems, allowing powering on and off subsystems to assure safety. One channel also is used to signal emergency-top to the RoboClaw motor controller.
* A touch screen display which displays the state of all the sensors.
* Communication with the RoboClaw motor controller.
  The main computer issues "cmd_vel" message to request motor operations and the custom board translates that into RoboClaw motor signals. It shapes motor velocities with acceleration/deceleration profiles. 
  The custom board computes a maximum distance to move so that if a continuous stream of motor commands isn't fed by the main computer, the RoboClaw will shut down the motors rather than leave them running in the last commanded velocity. The encoders are also independently sensed to see if they make sense according to the commanded motor velocities. This is important because if the motor encoder cables come loose from the RoboClaw board, the RoboClaw puts the motors into an uncontrolled high speed state. If the encoders as sensed by the custom board seem out of range, the RoboClaw will get an emergency stop request.

See: https://wimblerobotics.wimble.org/wp/
