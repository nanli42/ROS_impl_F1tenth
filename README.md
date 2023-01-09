# ROS packages for F1tenth race car

## Statement of purpose
ROS implementation of a generic system architecture for autonomous vehicles racing problem in head-to-head competition mode. The ROS nodes work in a Directed Acyclic Graph (DAG). The core components include:

- opponent detection (in package `f110_perception`)

- self-localization (in package `f110_perception`)

- controller (in packages `f110_nmpc`, `f110_central_ctrler`)


## Usage

1: compile the ROS packages
```bash
$ cd [this project]
$ catkin_make
$ source devel/setup.bash
```

2: start the simulator
```bash
$ cd [this project]/src/f110_gym_simulator
$ sudo ./build_docker.sh
$ sudo ./docker.sh
```
3: launch the rviz visualization and reset positions of two vehicles
```bash
$ roslaunch f110_central_ctrler rviz.launch
$ bash src/f110_perception/reset_pos.sh x_ego y_ego psi_ego x_opp y_opp psi_opp
```


4: launch the ego vehicle's modules
```bash
$ roslaunch f110_perception opp_detect.launch
$ roslaunch f110_perception pf_cpp.launch
$ roslaunch f110_nmpc nmpc_ctrler_ego.launch
```

5: launch the opponent vehicle's controller
```bash
$ roslaunch f110_nmpc nmpc_ctrler_opp.launch
```

6: start the competition
```bash
$ roslaunch f110_central_ctrler rviz.launch
```