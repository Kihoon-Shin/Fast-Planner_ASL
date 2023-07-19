# Todo list(for me)
take off height is 1m, mayb add param to change this(/plan_manage/script/trajectory_msg_converter_~.py)   

# Fast-Planner for px4

The orignal fast planner is https://github.com/HKUST-Aerial-Robotics/Fast-Planner   
in may case the main algorithm and concept is same but edited for px4 sitl and real flight

**px4**   
My drone use px4 with D435i, T265   
I added and edited some parts to run in px4 and check it in px4_sitl gazebo

# CONCEPT
This is simple   
(1) user - Set the destination   
(2) kino or topo planner gonna make the trajectory   
(3) pub the trajectory to mavros as /mavros/setpoint_raw/local   
(4) then px4 gonna fly   

 

In my case, I used https://github.com/mzahana/px4_fast_planner
and this one use 'movros_controllers', which was unstable in my drone

# Prerequisite
You should catkin build nlopt first in your catkin workspace.
- git clone https://github.com/ethz-asl/nlopt.git
- catkin build nlopt
- source devel/setup.bash

# INSTALL & SETTINGS
- cd ~/drone_ws/src # in my case 'drone_ws' and 'PX4-Autopilot' is located in HOME
- git clone https://github.com/Kihoon-Shin/Fast-Planner.git
- cd .. && catkin build && source devel/setup.bash
- cd src/Fast-Planner
- cp storage/1024_gazebo-classic_iris_depth_camera ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
- cp storage/depth_camera.sdf ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/depth_camera/
- cd ~/.ros/etc/init.d-posix/airframes
- cp ~/drone_ws/src/Fast-Planner/storage/1024_gazebo-classic_iris_depth_camera .
- cd ~/drone_ws/src/Fast-Planner/fast_planner/plan_manage/script
- sudo chmod 777 trajectory_msg_converter_raw.py 
- gedit ~/PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake 
>- **# about in line 103~130, add 'iris_depth_camera'**
- cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo-classic_iris_depth_camera
- gedit ~/drone_ws/src/Fast-Planner/uav_simulator/Utils/testbot_description/launch/testbot.launch
```
	<!--<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>-->
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

# RUN in gazebo
- cd ~/PX4-Autopilot
- source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
- roslaunch plan_manage px4_sitl_kino_replan.launch   

in another terminal
- rosrun mavros mavsafety arm 
- rosrun mavros mavsys mode -c OFFBOARD

- To command the drone to fly to a target pose, publish a single message to the /move_base_simple/goal topic as follows
```bash
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 19.0
    y: 15.0
    z: 3.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

# RUN in real drone
- cd ~/PX4-Autopilot
- source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
- roslaunch plan_manage px4_kino_replan.launch      
   
- arming and set the OFFBOARD mode

# ignition gazebo(working)
- cd ~/Firmware && make px4_sitl ignition
- roslaunch mavros px4.launch fcu_url:=udp://:14540@localhost:14557
- rosrun ros_ign_bridge parameter_bridge /depth_camera@sensor_msgs/Image@ignition.msgs.Image /rgb_camera@sensor_msgs/Image@ignition.msgs.Image /depth_camera/points@sensor_msgs/PointCloud2@ignition.msgs.PointCloudPacked

- rostopic echo /mavrolocal_position/pose
- rostopic echo --noar/depth_camera

- rosrun topic_tools transform /depth_camera /d435/depth/ima_rect_raw sensor_msgs/Image 'sensor_msgs.msg.Image(header=std_msgs.msg.Header(seq=m.header.seq,stamp=m.header.stamp,frame_id="camera_link"),height=m.height,width=m.width,encoding=m.encoding,is_bigendian=m.is_bigendian,step=m.step,data=m.data)' --import sensor_msgs std_msgs


# ERRORS

There were two main error when I tested and runned these

(1) Fast-planner is not working error
> when clicking and dragging for 2d nav goal then the error go up
![screensh](./image/fast_planner_error.png)
> https://blog.csdn.net/asd22222984565/article/details/126779002#comments_23784587

(2) mavros setpoint not working error in sitl
> if drone arms but not takes off then, I'm not sure but my solution was change the topic from /mavros/setpoint_raw/local to /mavros/setpoint_position/local   
> if you changes it, then the topic doesn't care about acc and vel, only position
> 
> the way to change the setpoint topic is edit the /Fast-Planner/fast_planner/plan_manage/launch/px4_sitl_kino_replan.launch
>  from <node pkg="plan_manage" name="traj_msg_converter" type="trajectory_msg_converter_raw.py" output="screen"/>
>  to <node pkg="plan_manage" name="traj_msg_converter" type="trajectory_msg_converter_pos.py" output="screen"/>

