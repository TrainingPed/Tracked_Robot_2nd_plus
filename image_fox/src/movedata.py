#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
x=[];x2=[];x3=[];x4=[];y=[];y4=[];y2=[];y3=[];th=[];th2=[];th3=[];th4=[];
fig, ax = plt.subplots()
xdata, ydata ,xdata2, ydata2 ,xdata3, ydata3 ,xdata4, ydata4 = [], [],[], [],[], [],[], []
x_limp,x_limn,y_limp,y_limn=5,-5,5,-5
ln1, = plt.plot([], [], 'b', animated=True)
ln2, = plt.plot([], [], 'r', animated=True)
ln3, = plt.plot([], [], 'g', animated=True)
ln4, = plt.plot([], [], 'y', animated=True)
f=np.zeros(1);

def init():
	ax.set_xlim(-3, 3)
	ax.set_ylim(-3, 3)
	ln1.set_data(xdata,ydata)
	ln2.set_data(xdata2,ydata2)
	ln3.set_data(xdata3,ydata3)
	ln4.set_data(xdata4,ydata4)
	return ln1,ln2,ln3,ln4,

def update(frame):
	xdata.append(x[frame])
	ydata.append(y[frame])
	xdata2.append(x2[frame])
	ydata2.append(y2[frame])
	xdata3.append(x3[frame])
	ydata3.append(y3[frame])
	xdata4.append(x4[frame])
	ydata4.append(y4[frame])
	global x_limp,x_limn,y_limn,y_limp
        if(x[frame]>x_limp):
	    x_limp=x[frame]+1
        if(x[frame]<x_limn):
	    x_limn=x[frame]-1
        if(y[frame]>y_limp):
	    y_limp=y[frame]+1
        if(y[frame]<y_limn):
	    y_limn=y[frame]-1
	ax.set_xlim(x_limn,x_limp)
	ax.set_ylim(y_limn,y_limp)
	#print('y[frame]=',y[frame])
	#print('ylimn=',y_limn)
	#print('\n')
	ln1.set_data(xdata,ydata)
	ln2.set_data(xdata2,ydata2)
	ln3.set_data(xdata3,ydata3)
	ln4.set_data(xdata4,ydata4)
	return ln1,ln2,ln3,ln4,

if __name__ == '__main__':
        filename=raw_input('Please enter the filename ...')
        print('Start ploting~')
        rospy.init_node('move_data_anim', anonymous=True)
        dir ='/home/nvidia/move_data/'
	x,y,th=np.loadtxt(fname=dir+filename+'ref.txt',delimiter='\t',unpack='True')
        x2,y2,th2=np.loadtxt(fname=dir+filename+'amcl.txt',delimiter='\t',unpack='True')
        x3,y3,th3=np.loadtxt(fname=dir+filename+'odom.txt',delimiter='\t',unpack='True')
        x4,y4,th4=np.loadtxt(fname=dir+filename+'fusion.txt',delimiter='\t',unpack='True')
	f = np.linspace(1,len(x)-1,len(x)-1)
  
    
	ani = FuncAnimation(fig, update, frames=f,
                    init_func=init, blit=True, interval = 2.5,repeat=False)
	plt.grid()
	ax.legend(['Reference','AMCL','Odometry','Fusion'],loc='best')
	plt.savefig('/home/nvidia/move_data/'+filename+'.png')
	plt.show()	
	rospy.spin()
