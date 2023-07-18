#!/usr/bin/env python3

import rospy,math
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal

def transition(msg):
    data = msg.data
    x = data/MoveBaseActionGoal.goal.target_pose.pose.position.x
    y = data/MoveBaseActionGoal.goal.target_pose.pose.position.y
    oz = data/MoveBaseActionGoal.goal.target_pose.pose.orientation.z
    ow = data/MoveBaseActionGoal.goal.target_pose.pose.orientation.w

    #四元数换算成theta
    theta = math.atan2(2*oz*ow,1)

    #发布
    goal_msg = Pose2D()
    goal_msg.x = x
    goal_msg.y = y
    goal_msg.theta = theta
    pub.publish(goal_msg)
    
rospy.init_node('robot_trans_goal_node')

sub = rospy.Subscriber('/nucros/cmd/goal', MoveBaseActionGoal, transition)
pub = rospy.Publisher('/robot_goal', Pose2D, queue_size=10)
