This simulator only supports pre-loaded traj file as input. Sample trajecory file can be found in $(hector_quadrotor_reference)/traj

### to use another traj file
specify the file name in hector_quadrotor_reference/src/sub.cpp.
Also, you need to update  the initial position of UAV by substracting the offset of pos input.

### To launch the simulator:
- source devel/sebup.bash
- roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch.launch
- roslaunch hector_quadrotor_reference hector_quadrotor_reference.launch



