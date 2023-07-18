#!/usr/bin/env python3

import rospy,math
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nucros_msgs.msg import RobotStatus

def transition(msg):
    data = msg.data
    x = data/PoseStamped.pose.pose.position.x
    y = data/PoseStamped.pose.pose.position.y
    oz = data/PoseStamped.pose.pose.orientation.z
    ow = data/PoseStamped.pose.pose.orientation.w

    #四元数换算成theta
    theta = math.atan2(2*oz*ow,1)

    #发布
    pose_msg = Pose2D()
    pose_msg.x = x
    pose_msg.y = y
    pose_msg.theta = theta

    pub.publish(pose_msg)

rospy.init_node('robot_trans_pose_node')

sub = rospy.Subscriber('/nucros/status/robot_status', RobotStatus, transition)
pub = rospy.Publisher('/robot_pose', Pose2D, queue_size=10)



