This simulator only supports pre-loaded traj file as input. Sample trajecory file can be found in $(hector_quadrotor_reference)/traj

### to use another traj file
specify the file name in hector_quadrotor_reference/src/sub.cpp.
Also, you need to update  the initial position of UAV by substracting the offset of pos input.

### To launch the simulator:
- source devel/sebup.bash
- roslaunch hector_quadrotor_demo indoor_carpark.launch
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


## to use keyboard control
- in controller.launch, launch controller/twist only
- in another terminal: rosrun teleop_twist_keyboard teleop_twist_keyboard.py
- rosservice call /engage
- play in the keyboard termial! press t to take off

## Update asus model
I found that if asus is mounted in front of the drone, the controller will diverge on the very beginning. So I mounted it
on top of the drone and make it symetry.
At the same time, I canceled the rotation on pitch axis to make the fov to the front. For now, it works well
with my old version EDT.
- TODO: However, I don't know how to stabilize the unbalanced setting and how to rotate the yaw angle of asus model...
