#!/usr/bin/env python
import rospy

from crio_ds.msg import DriverStation

if __name__ == "__main__":
    rospy.init_node("test", anonymous=True)
    pub = rospy.Publisher('/crio', DriverStation, queue_size=10)
    rate = rospy.Rate(1)
    rospy.loginfo("Sending msg")


    while not rospy.is_shutdown():
        msg = DriverStation()
        msg.enabled = True
        pub.publish(msg)
        rospy.loginfo("Sent Message")
        rate.sleep()