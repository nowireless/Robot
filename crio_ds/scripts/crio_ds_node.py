#!/usr/bin/env python
import rospy
import roslog
from crio.communication import DS
from crio.communication import DSException
from crio_ds.msg import DriverStationState
from std_msgs.msg import Bool


class CrioDSNode:
    def __init__(self):
        rospy.init_node('ds')
        team = rospy.get_param("team", 3081)
        rospy.loginfo("Team: %i", team)
        try:
            self.ds = DS(team)
        except DSException as e:
            rospy.logfatal("Could not start the Driver Station")
            return

        self.pub = rospy.Publisher("/ds_state", DriverStationState)
        rospy.Subscriber("/ds_enabled", Bool,  callback=CrioDSNode.callback, callback_args=(self,))

    def run(self):
        rospy.loginfo("Spinning")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.ds.state_lock.acquire()
            msg = DriverStationState()
            msg.packet = ds.packet_number
            msg.enabled = ds.state.enabled
            self.ds.state_lock.release()
            rate.sleep()
        ds.stop()

    def callback(self):
        pass

if __name__ == "__main__":
    roslog.init("DS")
    node = CrioDSNode()
    node.run()
