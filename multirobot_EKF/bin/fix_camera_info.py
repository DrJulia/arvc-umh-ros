#!/usr/bin/env python
import roslib; roslib.load_manifest('multirobot_EKF')
import rosbag
import rospy
import sys

with rosbag.Bag(sys.argv[2], 'w') as outbag:

   for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
	if topic == "/pioneer2/stereo/camera_right/camera_info":
	    #newK = (msg.P[0], msg.K[1], msg.P[2], msg.K[3], msg.P[5], msg.P[6], msg.K[6], msg.K[7], msg.K[8])
	    #msg.K = newK
	    newK = (1170.45, 0, 650.07471264304, 0, 1191.68, 456.247309332689, 0, 0, 1)
	    msg.K = newK
	    newP = (1170.45, 0, 650.07471264304, 0, 0, 1191.68, 456.247309332689, 0, 0, 0, 1, 0)
	    msg.P = newP
	outbag.write(topic, msg, t )
 

