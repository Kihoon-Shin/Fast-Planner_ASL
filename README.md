# Fast-Planner for Real Quadrotor with px4

The orignal fast planner is https://github.com/HKUST-Aerial-Robotics/Fast-Planner   
in may case the main algorithm and concept is same but edited for px4 sitl and real flight

**px4**   
My drone use px4 with D435i, T265   
I added and edited some parts to run in px4 and check it in px4_sitl gazebo

now kino planner is working but topo is not perfect 

# INSTALL & SETTINGS
- cd ~/catkin_ws/src # in my case 'catkin_ws' and 'PX4-Autopilot' is located in HOME
- git clone https://github.com/beomsu7/Fast-Planner 
- cd .. && catkin build && source devel/setup.bash
- cd Fast-Planner
- cp storage/1024_iris_depth_camera ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
- cp storage/depth_camera.sdf ~/PX4-Autopilot/Tools/sitl_gazebo/models/depth_camera/
- sudo vim ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake
about in line 103~130, add 'iris_depth_camera'
- cd ~/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default gazebo_iris_depth_camera
- source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
- export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
- roslaunch plan_manage px4_sitl_kino_replan.launch 

# ERRORS

There were two main error when I tested and runned these

(1) 
