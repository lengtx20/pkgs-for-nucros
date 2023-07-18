#!/usr/bin/env python3

import rospy,math

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

# rospy.init_node('move_ctrl_node')

# 控制设置
Kp = 0.05
Ki = 0.0005
Kd = 0.01
w0 = 0.3

# 常量设置
dt = 0.05
dt1 = 1/dt
N = 100
T = N*dt

# 误差设置
dtheta = 0.05
dr = 0.02

# 坐标设置
global x,y,theta,x_goal,y_goal,theta_goal,theta1_goal,r,r_goal,x_init,y_init
x = 0.0
y = 0.0
theta = 0.0
x_goal = 0.0
y_goal = 0.0
theta_goal = 0.0
theta1_goal = 0.0
r = 0.0
r_goal = 0.0
x_init = 0.0
y_init = 0.0

# 速度设置
global vx,wz
vx = 0.0
wz = 0.0

# 变量设置
E = 0.0
e = [0]
eprime = [0]
u = [0]
output = [0]
vel = [0]

global state
state = 0

def goal_set(msg1):
    state = 0
    x_goal,y_goal,theta_goal = msg1.data
    print("goal pose is: x=",x_goal," y= ",y_goal," theta=",theta_goal)

def vel_set(msg2):
    x,y,theta = msg2.data
    if state == 0:
        #初始化运动坐标和角度
        bizhi = (y_goal-y)/(x_goal-x)
        theta1_goal = math.atan(bizhi)      #或者180°相反，之后再补充
        r = 0
        x_init = x
        y_init = y
        r_goal = math.sqrt((x_goal-x)**2 + (y_goal-y)**2)
        vx = 0
        wz = 0

    elif state == 1:
        #直线运动前转动
        vx = 0
        if theta-theta1_goal >= dtheta or theta1_goal-theta >= dtheta:
            wz = w0
        else:
            wz = 0
            state = 2

    elif state == 2:
        #PID控制 需修改######################################
        wz = 0
        r = math.sqrt((x-x_init)**2 + (y-y_init)**2)
        if r_goal-r >= dr or r-r_goal >= dr:
            e.append(r_goal-r)                      #误差项
            E = E + e[-1]*dt                        #积分项
            eprime.append((e[-1]-e[-2])*dt1)        #微分项
            u.append(Kp*e[1]+Ki*E+Kd*eprime[-1])    #所需控制量
            vel.append(u*dt1)                       #所需速度
            vx = vel[-1]
        else:
            vx = 0
            state = 3

    elif state == 3:
        #直线运动后转动
        vx = 0
        if theta-theta_goal >= dtheta or theta_goal-theta >= dtheta:
            wz = w0
        else:
            wz = 0
            state = 4

    elif state == 4:
        #到达目标点
        vx = 0.0
        wz = 0.0

    vel_msg = Twist()
    vel_msg.linear.x = vx
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = wz

    pub.publish(vel_msg)
    print("vx=",vx," wz=",wz," state=",state," r=",r," r_goal=",r_goal)


def main():
    rospy.init_node('move_ctrl_node')
    sub1 = rospy.Subscriber('/robot_goal', Pose2D, goal_set)
    sub2 = rospy.Subscriber('/robot_pose', Pose2D, vel_set)
    # pub = rospy.Publisher('/nucros/cmd/cmd_vel', Twist, queue_size=10)
    rospy.spin()

#***************** main function ********************
if __name__ == "__main__":
    pub = rospy.Publisher('/nucros/cmd/cmd_vel', Twist, queue_size=10)
    main()


# rospy.spin()