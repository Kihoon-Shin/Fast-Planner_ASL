#!/usr/bin/env python

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
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

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, PoseStamped, queue_size=1)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        pose = PoseStamped()
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.traj_pub.publish(pose)

if __name__ == '__main__':
    obj = MessageConverter()
