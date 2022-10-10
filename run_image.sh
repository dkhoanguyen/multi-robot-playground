#!/bin/bash

docker run --privileged \
           --detach \
           --restart 'always' \
           --volume ${PWD}:/root/multi_robot_playground2/src/multi-robot-playground \
           --name mrp_dev_container \
           mrp_dev_image tail -f /dev/null
