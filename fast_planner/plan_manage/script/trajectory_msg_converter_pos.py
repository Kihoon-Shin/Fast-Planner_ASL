#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
origial file's Authors: Mohamed Abdelkader
https://github.com/mzahana/px4_fast_planner
reference : /px4_fast_planner/scripts/trajectory_msg_converter.py
"""

# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'mavros/setpoint_position/local')

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 1
        #q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 1
        
        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, PoseStamped, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
    

    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        self.pose = PoseStamped()
        self.pose.pose.position.x = msg.position.x
        self.pose.pose.position.y = msg.position.y
        self.pose.pose.position.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        #

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.traj_pub.publish(self.pose)
            rate.sleep()

if __name__ == '__main__':
    obj = MessageConverter()
    obj.run()


    
