This simulator only supports pre-loaded traj file as input. Sample trajecory file can be found in $(hector_quadrotor_reference)/traj

### to use another traj file
specify the file name in hector_quadrotor_reference/src/sub.cpp.
Also, you need to update  the initial position of UAV by substracting the offset of pos input.

### To launch the simulator:
- source devel/sebup.bash
- roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch
- roslaunch hector_quadrotor_reference hector_quadrotor_reference.launch

## Update velodyne simulator

For  now, pointcloud in rviz is relative to base_link.
### To launch the simulator:

- source devel/sebup.bash
- roslaunch hector_quadrotor_demo carpark_velodyne.launch
- roslaunch hector_quadrotor_reference hector_quadrotor_reference.launch

## Update multiple uav support
### To launch the simulator:
- source devel/sebup.bash
- roslaunch hector_quadrotor_demo carpark_2quad.launch
- roslaunch hector_quadrotor_reference ref_ns.launch
