#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np

x = []
y = []
th=[]
x2 = []
y2 = []
th2=[]
x3=[]
y3=[]
th3=[]
x4=[]
y4=[]
th4=[]
x1=[];y1=[]


def drawer():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    filename=raw_input('Please enter the filename ...')
    print('Start ploting~')
    rospy.init_node('move_data_plot', anonymous=True)

    dir ='/home/nvidia/move_data/'
    global t
    plt.ion()

    fig=plt.figure()
    fig2=plt.figure()
    fig3=plt.figure()
    fig4=plt.figure()
    ax = fig.add_subplot(111)
    bx = fig2.add_subplot(111)
    cx = fig3.add_subplot(111)
    dx = fig4.add_subplot(111)
    # plt.xlim(0,672)
    # plt.ylim=(0,376)
    x,y,th=np.loadtxt(fname=dir+filename+'ref.txt',delimiter='\t',unpack='True')
    x2,y2,th2=np.loadtxt(fname=dir+filename+'amcl.txt',delimiter='\t',unpack='True')
    x3,y3,th3=np.loadtxt(fname=dir+filename+'odom.txt',delimiter='\t',unpack='True')
    x4,y4,th4=np.loadtxt(fname=dir+filename+'fusion.txt',delimiter='\t',unpack='True')

    plot1, = ax.plot(x,y, 'b', label="Reference Path")
    plot2, = ax.plot(x2,y2, 'r', label="AMCL Path")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    plot21, = bx.plot(x,y, 'b', label="Reference Path")
    plot23, = bx.plot(x3,y3,'g',label='Odom Path')
    bx.set_xlabel("x (m)")
    bx.set_ylabel("y (m)")
    plot31, = cx.plot(x,y, 'b', label="Reference Path")
    plot34, = cx.plot(x4,y4,'y',label='Fusion Path')
    cx.set_xlabel("x (m)")
    cx.set_ylabel("y (m)")
    plot41, = dx.plot(x,y, 'b', label="Reference Path")
    plot42, = dx.plot(x2,y2, 'r', label="AMCL Path")
    plot43, = dx.plot(x3,y3,'g',label='Odom Path')
    plot44, = dx.plot(x4,y4,'y',label='Fusion Path')
    dx.set_xlabel("x (m)")
    dx.set_ylabel("y (m)")
    #plt.draw()

    ax.grid()
    ax.xaxis.set_major_locator(plt.MultipleLocator(5))
    ax.yaxis.set_major_locator(plt.MultipleLocator(5))

    bx.grid()
    bx.xaxis.set_major_locator(plt.MultipleLocator(5))
    bx.yaxis.set_major_locator(plt.MultipleLocator(5))
    cx.grid()
    cx.xaxis.set_major_locator(plt.MultipleLocator(5))
    cx.yaxis.set_major_locator(plt.MultipleLocator(5))
    dx.grid()
    dx.xaxis.set_major_locator(plt.MultipleLocator(5))
    dx.yaxis.set_major_locator(plt.MultipleLocator(5))
    #plt.show()
    # spin() simply keeps python from exiting until this node is stopped
    #n=1
    #left,right=-5,25
    #top,down=5,-20
    #ax.set_xlim(left, right)
    #ax.set_ylim(down,top)
    #bx.set_xlim(left, right)
    #bx.set_ylim(down,top)
    #cx.set_xlim(left, right)
    #cx.set_ylim(down,top)
    #dx.set_xlim(left, right)
    #dx.set_ylim(down,top)

   # ax.axis('equal')
    #bx.axis('equal')
    #cx.axis('equal')
    #dx.axis('equal')


    ax.autoscale(enable=True,axis='both')
    bx.autoscale(enable=True,axis='both')
    cx.autoscale(enable=True,axis='both')
    dx.autoscale(enable=True,axis='both')

    #rospy.spin()
    ax.legend(['Reference','AMCL'],loc='best')
    bx.legend(['Reference','Odometry'],loc='best')
    cx.legend(['Reference','Fusion'],loc='best')
    dx.legend(['Reference','AMCL','Odometry','Fusion'],loc='best')
    #ax.legend(['Reference','Odometry'],loc='best')
    #ax.legend(['Reference','Fusion'],loc='best')
    #ax.legend(['Reference','AMCL','Odometry','Fusion'],loc='best')
    #plt.ioff()
    fig.savefig('/home/nvidia/move_data/'+filename+'amcl.png')
    fig2.savefig('/home/nvidia/move_data/'+filename+'odom.png')
    fig3.savefig('/home/nvidia/move_data/'+filename+'fusion.png')
    fig4.savefig('/home/nvidia/move_data/'+filename+'all.png')


    plt.show()

#    plt.savefig('/home/nvidia/move_data/'+filename+'.png')
    #rospy.spin()


if __name__ == '__main__':
    drawer()
