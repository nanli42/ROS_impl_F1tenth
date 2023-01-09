#!/bin/bash
docker run --ulimit rtprio=99 -it --name=f1tenth_gym_container --rm --net=host f1tenth_gym
#docker run --ulimit rtprio=99 -it --name=f1tenth_gym_container --rm --env ROS_MASTER_URI=http://192.168.55.1:11311 --env ROS_HOSTNAME=192.168.55.100 f1tenth_gym 
#docker run --ulimit rtprio=99 -it --name=f1tenth_gym_container --rm --env ROS_MASTER_URI=http://192.168.55.100:11311 --env ROS_HOSTNAME=192.168.55.100 --net=host f1tenth_gym 
#docker run --ulimit rtprio=99 -it --name=f1tenth_gym_container --rm --env ROS_MASTER_URI=http://192.168.55.1:11311 --env ROS_HOSTNAME=192.168.55.100 --env ROS_IP=192.168.55.100 --privileged --net=host f1tenth_gym 
