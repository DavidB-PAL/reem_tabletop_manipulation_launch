### ROS Node: reem_tabletop_manipulation_launch
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>
<br>
Launch and configuration files for doing Tabletop Object Manipulation with the REEM robot. 

Works with the simulated and real robot.

There are 2 monolithic launch files. One launches the required nodes to detect/recognize objects, and other does that + loads the extra nodes for object manipulation. <br>
The demo code shows how to return information about the table, detected object PointCloud clusters and recognized object models. It also shows how to grasp the nearest object using a specific arm.
<br>

<br>
Note: This uses alternative YAML files for arm kinematics and planning, so we can specify the grasping frame as the tip link. Without these, things will fail. <br>
Also, the Interpolated IK node uses the IK Services that were loaded from the launch file. But object_manipulator loads the IK separately as a plugin, and ignores any alternative parameters!

ToDo: 
 - Debug object_manipulator to find out why it is ignoring the +z lift direction (upwards). Maybe a problem with TF frame transform.
 - Test grasping with right arm.
<br>

<br>
**Required ROS packages:** <br>
reem_common     (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_simulation (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_arm_navigation (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
reem_kinematics (DavidB-PAL fork, not yet merged with Master 15/3/13) 

arm_kinematics_constraint_aware (DavidB-PAL fork) <br>
move_arm (DavidB-PAL fork) <br>
household_objects_database (DavidB-PAL fork) <br>
tabletop_object_detector (DavidB-PAL fork) <br>
tabletop_collision_map_processing (DavidB-PAL fork)

pointcloud_snapshotter <br>
fake_controllers_list <br>
reem_manipulation_worlds <br>
reem_perception_launch <br>
reem_head_scan_action <br>
reem_move_arm_action
<br>

<br>
**Usage:** <br>

Tabletop Perception demo:

$ export USE_RGBD_SENSOR=true <br>
$ roslaunch reem_manipulation_worlds reem_high_white_table_manipulation.launch <br>
$ roslaunch reem_tabletop_manipulation_launch tabletop_perception.launch sim:=true use_snapshotter:=true <br>
$ roslaunch reem_tabletop_manipulation_launch rviz.launch <br>
$ rosrun reem_tabletop_manipulation_launch tabletop_perception_test <br>
Check the output returned by the test. <br>
Also view the various visualizations in Rviz. <br>
<br>

<br>
Tabletop Object Manipulation demo:

$ export USE_RGBD_SENSOR=true <br>
$ roslaunch reem_manipulation_worlds reem_high_white_table_manipulation.launch <br>
$ roslaunch reem_tabletop_manipulation_launch tabletop_manipulation.launch sim:=true use_snapshotter:=true load_kinematics:=true load_move_arm:=true load_move_arm_warehouse:=false <br>
$ roslaunch reem_tabletop_manipulation_launch rviz.launch

Move both arms so they are not in collision with the body: <br>
$ rosrun reem_move_arm_action move --arm=left --pose=home_to_init; rosrun reem_move_arm_action move --arm=right --pose=home_to_init; 

Optional, pre-scan the table and move arm to pre-grasp pose: <br>
$ rosrun reem_head_scan_action scan_table; rosrun reem_move_arm_action move --arm=left --pose=elbow_back

Run the test, while watching the output in the 2nd terminal and in Rviz: <br>
$ rosrun reem_tabletop_manipulation_launch tabletop_grasping_test 

To manually lift the object: <br>
$ rosrun reem_move_arm_action move --arm=left --pose=grasp_lift






