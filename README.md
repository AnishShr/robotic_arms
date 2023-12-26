# robotic_arms

#### Xamr7 Pick and Place

To run the pick akd place xarm7 manipulator robot, follow the isntructions as follows:    

- In a terminal run the following command:   
```
roslaunch xarm7_gripper_moveit_config demo.launch   
```

- In another terminal, run the following node:   
```
rosrun movegroup_interface_demo pick_and_place_xarm7   
``` 

Make sure that all required packages are built and sourced. In order to do that, run the following commands from the root of the catkin ws:   
```
catkin build   
source devel/setup.bash   
```


#### Dual UR5 Emika-Franka-Panda coordnianted Manipulation
irst, the launch file needs to be run which will launch the RViz with the 2 robots sharing the same cylindrical platform. To run the launch file, run the follwoing command from the root of the catkin ws:   
```
roslaunch dual_ur5_panda_moveit_config demo.launch   
```

After the RViz window launches, there should be a blur cylindrical (circular base) platform, with the 2 robots visualized. To make the robots move, run the following command fromt he root of the catkin ws:   
```
rosrun movegroup_interface_demo dual_pose_goal   
```

** Make sure to build and source the catkin ws:   
(from the root of the catkin ws)   
```
catkin build
source devel/setup.bash
```