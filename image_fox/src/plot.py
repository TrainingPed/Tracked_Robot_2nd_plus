#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
x = []
y = []
x2 = []
y2 = []
t = 0


def dcallback(d):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %d", d.data)
    global x
    global t
    global y
    d_t = d.data
    x.append(t)
    y.append(d_t)
#    print(" y: ", y)
def cntcallback(cnt21):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d", cnt21.data)
    global x2
    global t
    global y2
    cnt_t = cnt21.data
    x2.append(t)
    y2.append(cnt_t)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    filename=raw_input('Please enter the filename ...')
    print('Start Recording~')
    rospy.init_node('plot', anonymous=True)

    rospy.Subscriber("d", Int32, dcallback)
    rospy.Subscriber("cnt21", Int32, cntcallback)
    
    global t
    plt.ion()
    
    fig=plt.figure()
    ax = fig.add_subplot(111)
    # plt.xlim(0,672)
    # plt.ylim=(0,376)
    plot1, = ax.plot(x, y, 'b', label="d")
    #plot1, = ax.scatter(x, y)
    plot2, = ax.plot(x2, y2, 'r', label="cnt21")     
    # ax.plot(x, y, 'b', label='Odom')
    plt.xlabel("frame")
    plt.ylabel("depth(cm)")
    # plt.draw()
    plt.grid()
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        t+=1
        # print("test"3)
        plt.autoscale(enable=True,axis='both')
        plot1.set_data(x,y)	
        plot2.set_data(x2,y2)
        ax.set_xlim(0,t)
        ax.set_ylim(0,800)
        ax.legend(['d','cnt'])
        fig.canvas.draw()

    rospy.spin()
    plt.savefig('/home/nvidia/depth_plot/depth_plot_'+filename+'.png')


if __name__ == '__main__':
    listener()
