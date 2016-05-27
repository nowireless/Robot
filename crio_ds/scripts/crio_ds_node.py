#!/usr/bin/env python
import rospy
from crio.communication import DS
from crio.communication import DSException
from crio_ds.msg import DriverStation

def callback(data):
    rospy.loginfo(type(data))
    ds.state_lock.acquire()
    ds.state.enabled = data.enabled
    ds.state_lock.release()


def listener():
    rospy.init_node('ds')
    team = rospy.get_param("team", 3081)
    rospy.loginfo("Team: %i", team)

    ds = None
    try:
        ds = DS(team)
    except DSException as e:
        #rospy.logfatal()
        #print e
        return

    rospy.Subscriber("/crio", DriverStation, callback)
    rospy.loginfo("Spinning")
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()

    ds.stop()
if __name__ == "__main__":
    listener()
#Report error mesage, and use ros param
#ds = DS(3081)
