#!/usr/bin/env bash
#docker run --rm --privileged  -p 6080:80 --shm-size=8192m -e RESOLUTION=1920x1080 -v ~/raven_ws:/home/ubuntu/raven_ws ros2_iron:latest
docker run -it --runtime=nvidia --gpus all -v /dev/:/dev/ -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/snd -e ALSADEV=hw:0,0 -v /home/ros/raven_ws:/home/ros/raven_ws --workdir /home/ros/raven_ws  --user 1000:1000 ros2_iron