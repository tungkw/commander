# Dependency

## python
[ikfastpy](https://github.com/andyzeng/ikfastpy)
```
python setup.py install [--user]
```

## ROS
[Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

# install 
as a ROS package

# Start

## run node
```
rosrun commander controller_command.py
```
what this node do:
1. subscribe joint state from 'scaled_pos_joint_traj_controller/state'
2. provide a ROS service deal with new target request
3. build trajactory using ikfastpy
4. send trajactory to 'scaled_pos_joint_traj_controller/command'

## send request
2. script
- msg format
```
float64[] joint_positions     # target position
float64 v_scale               # velocity (rad/sec) [recommend 0.1]
float64 duration_low_bound    # lower bound of duration (sec) from current position to target position, prevent too fast movement [recommend 1.0]
float64 start_duration        # set for dealing delay [recommend 0.5]
---
int8 success                  # whether ikfast compute inverse kinematics successfully. if false, no trajectory is made. 
```
- shell demo
```
rosservice call /command_move_to [-0.2,-0.6,0.43,1.64,2.09,-0.005] 0.2 1.0 0.5
```
