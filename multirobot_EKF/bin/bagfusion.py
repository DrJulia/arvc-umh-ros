#!/usr/bin/env python
import roslib; roslib.load_manifest('multirobot_EKF')
import rosbag
import rospy
import sys

with rosbag.Bag(sys.argv[3], 'w') as outbag:
   inittime = rospy.Time(0)
   difftime = rospy.Time(0)
   firsttime = True
   for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
	if firsttime:
		firsttime = False
		inittime = t
        if topic == "tf" and msg.transforms:
	    msg.transforms[0].header.stamp = msg.transforms[0].header.stamp
            outbag.write(topic, msg, t )
        else:
	    msg.header.stamp = msg.header.stamp
            outbag.write(topic, msg, t )
   firsttime = True
   for topic, msg, t in rosbag.Bag(sys.argv[2]).read_messages():
	if firsttime:
		firsttime = False
		difftime = t - inittime
        if topic == "tf" and msg.transforms:
	    msg.transforms[0].header.stamp = msg.transforms[0].header.stamp - difftime
            outbag.write(topic, msg, t - difftime)
        else:
	    msg.header.stamp = msg.header.stamp - difftime
            outbag.write(topic, msg, t - difftime)
   outbag.reindex
