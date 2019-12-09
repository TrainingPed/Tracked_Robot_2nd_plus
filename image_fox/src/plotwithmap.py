#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import rospy
import cv2
from matplotlib.animation import FuncAnimation
x=[];x2=[];x3=[];x4=[];y=[];y4=[];y2=[];y3=[];th=[];th2=[];th3=[];th4=[];
#fig, ax = plt.subplots()
xdata, ydata ,xdata2, ydata2 ,xdata3, ydata3 ,xdata4, ydata4 = [], [],[], [],[], [],[], []
x_limp,x_limn,y_limp,y_limn=5,-5,5,-5
#f=np.zeros(1);

if __name__ == '__main__':
        filename=raw_input('Please enter the filename ...')
        print('Start ploting~')
        rospy.init_node('move_data_anim', anonymous=True)
        dir ='/home/nvidia/move_data/'
	x,y,th=np.loadtxt(fname=dir+filename+'ref.txt',delimiter='\t',unpack='True')
        x2,y2,th2=np.loadtxt(fname=dir+filename+'amcl.txt',delimiter='\t',unpack='True')
        x3,y3,th3=np.loadtxt(fname=dir+filename+'odom.txt',delimiter='\t',unpack='True')
        x4,y4,th4=np.loadtxt(fname=dir+filename+'fusion.txt',delimiter='\t',unpack='True')
	#f = np.linspace(1,len(x)-1,len(x)-1)
  	map_num=raw_input('Please enter the map number (1~5)')	
	map_dir='/home/nvidia/catkin_ws/src/image_fox/maps/ntnu'+map_num+'f.png'
	#map_dir='/home/nvidia/Downloads/qt_icon.png'	
	img=cv2.imread(map_dir)
	cv2.circle(img, (500,492),1, (0, 0, 255),1)
	cropimg=img[300:650,400:800]
	
    	#img = plt.imread(map_dir)
	fig, ax = plt.subplots()

	ax.imshow(cv2.cvtColor(cropimg, cv2.COLOR_BGR2RGB),extent=[-20, 60, -31.5, 37.5])

	plt.ion()
	ax.plot(x,y)
	ax.plot(x2,y2)
	ax.plot(x3,y3)
	ax.plot(x4,y4)
#	ani = FuncAnimation(fig, update, frames=f,init_func=init, blit=True, interval = 2.5,repeat=False)
	plt.grid()
	ax.legend(['Reference','AMCL','Odometry','Fusion'],loc='best')
	plt.savefig('/home/nvidia/move_data/'+filename+'_map.png')
	plt.show()	
	rospy.spin()
