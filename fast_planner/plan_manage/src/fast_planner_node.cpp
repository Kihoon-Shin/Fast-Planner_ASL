// /**
// * This file is part of Fast-Planner.
// *
// * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
// * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
// * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
// * If you use this code, please cite the respective publications as
// * listed on the above website.
// *
// * Fast-Planner is free software: you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * Fast-Planner is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU Lesser General Public License
// * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
// */



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;



int main(int argc, char** argv) {

  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");


  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
          ("/mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("/mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("/mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("/mavros/set_mode");

  ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    break;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

// Takeoff //

  int planner;
  nh.param("planner_node/planner", planner, -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;

  if (planner == 1) {
    kino_replan.init(nh);
  } else if (planner == 2) {
    topo_replan.init(nh);
  }

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}

// #include <visualization_msgs/Marker.h>

// #include <plan_manage/kino_replan_fsm.h>
// #include <plan_manage/topo_replan_fsm.h>

// #include <plan_manage/backward.hpp>

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>

// namespace backward {
// backward::SignalHandling sh;
// }

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "offb_node");
//     ros::NodeHandle nh("~");

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//             ("mavros/state", 10, state_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//             ("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//             ("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//             ("mavros/set_mode");

//     //the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);

//     // wait for FCU connection
//     while(ros::ok() && !current_state.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }

//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 2;

//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request = ros::Time::now();

//     while(ros::ok()){
//         if( current_state.mode != "OFFBOARD" &&
//             (ros::Time::now() - last_request > ros::Duration(5.0))){
//             if( set_mode_client.call(offb_set_mode) &&
//                 offb_set_mode.response.mode_sent){
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         } else {
//             if( !current_state.armed &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0))){
//                 if( arming_client.call(arm_cmd) &&
//                     arm_cmd.response.success){
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         local_pos_pub.publish(pose);

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// } 