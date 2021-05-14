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
from mavros_msgs.msg import PositionTarget

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'mavros/setpoint_raw/local')

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, PositionTarget, queue_size=3)

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)

        rospy.spin()

    def fastPlannerTrajCallback(self, msg):
        # position and yaw
        pose = PositionTarget()
        pose.position.x = msg.position.x
        pose.position.y = msg.position.y
        pose.position.z = msg.position.z
        
        pose.velocity.x = msg.velocity.x
        pose.velocity.y = msg.velocity.y
        pose.velocity.z = msg.velocity.z

        pose.acceleration_or_force.x = msg.acceleration.x
        pose.acceleration_or_force.y = msg.acceleration.y
        pose.acceleration_or_force.z = msg.acceleration.z


        pose.yaw = msg.yaw


        self.traj_pub.publish(pose)

if __name__ == '__main__':
    obj = MessageConverter()
