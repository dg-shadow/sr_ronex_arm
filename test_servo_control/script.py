#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float64
from time import sleep


class Script():
    def __init__(self):
        self.pub = []
        self.pub.append( rospy.Publisher("/joint_1_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_2_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_3_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_4_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_5_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/gripper_controller/command", Float64) )

        self.start_values = []
        self.start_values.append(0)
        self.start_values.append(0.1)
        self.start_values.append(pi/2)
        self.start_values.append(-pi/2)
        self.start_values.append(0)
        self.start_values.append(0.02)

        self.end_values = []

        self.end_values.append(0)
        self.end_values.append(pi/2)
        self.end_values.append(0.1)
        self.end_values.append(0)
        self.end_values.append(pi/2)
        self.end_values.append(0.02)

      
    def run (self):
        rospy.init_node("test_publisher")
        for x in range(0,3):
            self.pub[x].publish(self.start_values[x]);

        rospy.sleep(3)

        t = 0

        while (t <= 5):
            for x in range(0,3):
                value = t/5*(self.end_values[x] - self.start_values[x]) + self.start_values[x]
                self.pub[x].publish(value)
                t = t + 0.1
                rospy.sleep(.1)
        rospy.sleep(3)
        while (t >= 0):
            for x in range(0,3):
                value = t/5*(self.end_values[x] - self.start_values[x]) + self.start_values[x]
                self.pub[x].publish(value)
                t = t - 0.1
                rospy.sleep(.1)
 
        rospy.sleep(3)

        
        
    
        
if __name__ == '__main__':
    
    m = Script()
    m.run()
