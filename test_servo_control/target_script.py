#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float64, Float64MultiArray
from time import sleep

class SrRonexArmTargetRepublisher():
    def __init__(self):
        self.publisher = rospy.Publisher("/arm_targets", Float64MultiArray)
        self.targets = [0,0,0,0,0,0.001]

     
    def publish(self):
        to_send = Float64MultiArray()
        to_send.data = self.targets
        self.publisher.publish(to_send)

    def run (self):
        rospy.init_node("command_publisher")
        self.targets[0] = .5
        self.targets[1] = .5
        self.targets[2] = .5
        self.publish()

        rospy.sleep(5)
        
        self.targets[0] = 0
        self.targets[1] = .1
        self.targets[2] = .1
        self.publish()

        rospy.sleep(5)

        self.targets[0] = -.5
        self.targets[1] = .5
        self.targets[2] = .5
        self.publish()

        rospy.sleep(5)

        self.targets[0] = 0
        self.targets[1] = .1
        self.targets[2] = .1
        self.publish()

        rospy.sleep(5)
    
        
if __name__ == '__main__':
    
    tp = SrRonexArmTargetRepublisher()
    tp.run()
