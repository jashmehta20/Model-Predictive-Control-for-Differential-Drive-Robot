#!/usr/bin/env python

# import ROS Libraries
from typing import Counter
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist

# import Matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math

# global variable decalration 
global robot_pose,qd,err,v,w,t
robot_pose = Pose2D()
qd = Pose2D()
err = Pose2D()
v = Twist()
w = Twist()
t = 0.0
ct=0
xl = []
yl = [] 
xd = []
yd = [] 
xe = [] 
ye = []
th = []  
vr = []
wr = []  
s = []

# callback functions to recive robot cooridnates, velocities
def pose_callback(msg):
    global robot_pose,v,w
    
    robot_pose.x = msg.pose.pose.position.x
    robot_pose.y = msg.pose.pose.position.y
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
    
# callback functions to get desired cooridnates
def desired_callback(msg):
    global qd,t
    
    qd.x = msg.x
    qd.y = msg.y
    t = t + 0.1

# callback functions to get Tracking errors
def errors_callback(msg):
    global err
    
    err.x = msg.x
    err.y = msg.y
    err.theta = msg.theta 


# animate functions to plot data
def plot_traj(i):
    xd.append(float(qd.x))
    yd.append(float(qd.y))
    ax1.plot(xd,yd, 'r',linestyle='dashed',label="reference trajectory",lw=2)  

    xl.append(float(robot_pose.x))
    yl.append(float(robot_pose.y))
    ax1.plot(xl,yl, 'k',label="current trajectory",lw=2)  
    
    if i==0:
        ax1.grid(True)
        ax1.legend()
    
def plot_error(i):

    xe.append(float(err.x))
    ye.append(float(err.y))
    th.append(float(err.theta))
    s.append(float(t))

    ax2.plot(s ,xe,'k',label=r"$x_e$" ,lw=2)  
    ax2.plot(s ,ye,'r',label=r"$y_e$" ,lw=2)  
    ax2.plot(s ,th,'b',label=r'$\theta_e$' ,lw=2) 

    ax2.grid(True)
    if i==0:
        ax2.legend()

    
if __name__ == '__main__':

    # initialization of ROS node
    rospy.init_node('back_cir', anonymous=True) #make node 
    sub = rospy.Subscriber("/odom", Odometry, pose_callback, queue_size=1000)
    sub1 = rospy.Subscriber("/desired_traj_pub", Pose2D, desired_callback, queue_size=1000)
    sub2 = rospy.Subscriber("/errors_pub", Pose2D, errors_callback, queue_size=1000)

    fig1 = plt.figure()
    fig2 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax2 = fig2.add_subplot(111)
    ax1.set_ylabel("Y Coordinates")
    ax1.set_xlabel("X coordinates")

    ax2.set_ylabel("Errors")
    ax2.set_xlabel("Time in seconds")

    ani1 = animation.FuncAnimation(fig1, plot_traj, interval=100)
    ani2 = animation.FuncAnimation(fig2, plot_error, interval=100)
    plt.show()
    rospy.spin()






