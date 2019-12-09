#!/usr/bin/env python
# license removed for brevity
import rospy
import rostopic
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

rospy.init_node('talker', anonymous=True)
#topics='/test'
topics='/zed/rgb/image_rect_color/compressed'
r = rostopic.ROSTopicHz(-1)
s = rospy.Subscriber(topics, rospy.AnyMsg, r.callback_hz, callback_args=topics)
rate_array=[]
times=[]
rate = rospy.Rate(10) # 10hz
init_time = datetime.now()
fig=plt.figure()
ax = fig.add_subplot(111)
i=0
while not rospy.is_shutdown():
    ret = r.get_hz(topics)
    i+=1
    if ret is None:
        print("no new messages")
    else:
        rates, min_delta, max_delta, std_dev, window = ret
        now = datetime.now()
        sec=(now-init_time).total_seconds()
        print('average rate:%.3f time:%.3f\n'%(rates,sec))
        rate_array.append(rates)
        #times.append(i)

        times.append(sec)

    #r.print_hz(['/test'])
    plt.plot(times,rate_array,'b',label='fps')
    #plot1, = ax.plot(times,rate_array,'b',label='fps')
    plt.xlabel('times(sec)')
    plt.ylabel('fps')
    rate.sleep()
fig.autofmt_xdate()
#ax.legend(['image'])
plt.show()

