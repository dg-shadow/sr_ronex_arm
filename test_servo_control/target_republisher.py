#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float64, Float64MultiArray


class SrRonexArmTargetRepublisher():
    def __init__(self):
        self.pub = []
        self.pub.append( rospy.Publisher("/joint_1_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_2_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_3_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_4_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_5_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/gripper_controller/command", Float64) )

        self.positions = [0,0,0,0,0,.001]

        self.subscriber = rospy.Subscriber("/arm_targets", Float64MultiArray, self.subscriber_callback)

    def subscriber_callback(self, msg):
        assert ( len(msg.data) == len( self.positions ) )
        self.positions = msg.data
        self.publish()
      
    def publish(self):
        for x in range (0, len( self.positions ) ):
            self.pub[x].publish(self.positions[x])

    def run (self):
        rospy.init_node("target_republisher")

        rospy.spin()

       
        
    
        
if __name__ == '__main__':
    
    tp = SrRonexArmTargetRepublisher()
    tp.run()
