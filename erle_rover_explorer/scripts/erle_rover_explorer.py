#!/usr/bin/env python
import rospy

import cv2
import numpy as np
import math
import mavros

from sensor_msgs.msg import LaserScan
from mavros.msg import OverrideRCIn

pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

def callback(data):
    frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_max
    Vx = 250
    Vy = 250
    for r in data.ranges:
        if r == float ('Inf'):
            r = data.range_max
        x = math.trunc( (r * 10)*math.cos(angle + (-90*3.1416/180)) )
        y = math.trunc( (r * 10)*math.sin(angle + (-90*3.1416/180)) )
        cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),1)
        Vx+=x
        Vy+=y
        angle= angle - data.angle_increment

    cv2.line(frame,(250, 250),(250+Vx,250+Vy),(0,0,255),3)
    cv2.circle(frame, (250, 250), 2, (255, 255, 0))
    ang = -(math.atan2(Vx,Vy)-3.1416)*180/3.1416
    if ang > 180:
        ang -= 360
    cv2.putText(frame,str(ang)[:10], (50,400), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255))

    cv2.imshow('frame',frame)
    cv2.waitKey(1)
    
    yaw = 1500 + ang * 40 / 6
    throttle = 1900
        
    msg = OverrideRCIn()
    msg.channels[0] = yaw
    msg.channels[1] = 0
    msg.channels[2] = throttle
    msg.channels[3] = 0
    msg.channels[4] = 0
    msg.channels[5] = 0
    msg.channels[6] = 0
    msg.channels[7] = 0
    pub.publish(msg)   


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    
    rospy.Subscriber("/scan", LaserScan,callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()
