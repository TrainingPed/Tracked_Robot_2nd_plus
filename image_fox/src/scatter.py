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


def midcallback(mid):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %d", d.data)
    global x2
    global y2
    midxy=mid.data
    x2.append(midxy[0])
    y2.append(midxy[1])
#    print(" y: ", y)
def mid5callback(mid5):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d", cnt21.data)
    #print('mid5=',mid5.data)
    global x
    global y
    mid_5 = mid5.data
    x.append(mid_5[0])
    y.append(mid_5[1])

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
     
    filename=raw_input('Please enter the filename ...')
    print('Start Recording~')

    rospy.init_node('scatter', anonymous=True)

    # rospy.Subscriber("d", Int32, dcallback)
    rospy.Subscriber("midxy", Int32MultiArray, midcallback)
    rospy.Subscriber("mid5", Int32MultiArray, mid5callback)
    
    global t
    plt.ion()
    
    fig=plt.figure()
    ax = fig.add_subplot(111)
    # plt.xlim(0,672)
    # plt.ylim=(0,376)

    plot1 = ax.scatter(x, y,c='blue')
    plot2 = ax.scatter(x2, y2,c='r')
    # plot2, = ax.plot(x2, y2, 'r', label="cnt21")     
    # ax.plot(x, y, 'b', label='Odom')
    plt.xlabel("x")
    plt.ylabel("y")
    # plt.draw()
    plt.grid()
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        t+=1
        # print("test"3)
        plt.autoscale(enable=True,axis='both')
        plot1.set_offsets(np.c_[x,y])
        plot2.set_offsets(np.c_[x2,y2])
        # plot2.set_data(x2,y2)
        ax.set_xlim(0,672)
        ax.set_ylim(0,376)
        ax.legend(['mid5','mid21'])
        fig.canvas.draw()

    rospy.spin()
    plt.savefig('/home/nvidia/depth_plot/mid5_scatter_'+filename+'.png')


if __name__ == '__main__':
    listener()
