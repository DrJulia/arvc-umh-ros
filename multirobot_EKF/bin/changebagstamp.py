#!/usr/bin/env python
import roslib; roslib.load_manifest('multirobot_EKF')
import rosbag
import rospy

with rosbag.Bag('output.bag', 'w') as outbag:
#   inittime = rospy.Time(0)
   inittime = rospy.Time(1604.836675)
   for topic, msg, t in rosbag.Bag('input.bag').read_messages():
	#print msg
        if topic == "tf" and msg.transforms:
	    msg.transforms[0].header.stamp = msg.transforms[0].header.stamp - inittime
            outbag.write(topic, msg, t - inittime)
        else:
	    msg.header.stamp = msg.header.stamp - inittime
            outbag.write(topic, msg, t - inittime)

