#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import Float64, Float64MultiArray
import thread


class SrRonexArmTargetCommander():
    def __init__(self):
        self.pub = []
        self.pub.append( rospy.Publisher("/joint_1_position_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_2_position_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_3_position_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_4_position_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/joint_5_position_controller/command", Float64) )
        self.pub.append( rospy.Publisher("/gripper_position_controller/command", Float64) )

        self.positions = [0,0,0,0,0,.001]

#        self.subscriber = rospy.Subscriber("/arm_targets", Float64MultiArray, self.subscriber_callback)

    def subscriber_callback(msg):
        assert ( self.msg.size == len( self.positions ) )
        self.positions = self.msg.data
        self.publish()

      
    def publish(self):
        for x in range (0, len( self.positions ) ):
            self.pub[x].publish(self.positions[x])

    def go(self, target, time):
       self.positions = target
       self.publish()
       rospy.sleep(time)


    def interpolate(self, target, time, sleep):
       current = self.positions
       x = 0
       dt = .01

       while x < time :
           for n in range (0,len(self.positions)):
               self.positions[n] = current[n] + (target[n]-current[n])*x/time
           self.publish()
           x = x + dt
           rospy.sleep(dt)

       rospy.sleep(sleep)



    def run (self):
        self.go([0.681,1.365, 1.22,0.65,0.68,0.001], 5 )
        self.interpolate([0.681,1.365, 1.22,0.65,0.68,0.01], 2, 1 )
        self.interpolate([0.694,1.59,  1.05,0.65,0.68,0.01], 8, 3)
        self.interpolate([0.694,1.59, 1.05,0.65,0.68,0.001], 2,3)
#        self.interpolate ([0.3,0.3,0.3,.3,.3,.3], 10)

        self.interpolate([0.681,1.365, 1.22,0.65,0.68,0.001], 3,3)

        rospy.sleep(1)

    def dum(self) :
        self.go([0.681,1.365, 1.22,0.65,0.68,0.001], 5)

        self.go([0.681,1.365, 1.22,0.655,0.68,0.01], 1)
        self.go([0.698,1.599, 1.02,0.652,0.70,0.01], 5)
        self.go([0.698,1.599, 1.02,0.652,0.70,0.001], 1)
        self.go([0.6808,1.365,1.22,0.655,0.68,0.001], 4)

        self.go([-0.48, 1.26, 1.97,-0.1,-0.48,0.001], 2)
        self.go([-0.48, 1.54, 1.82,-0.2,-0.48,0.001], 3)
        self.go([-0.48, 1.54, 1.82,-0.2,-0.48,0.01], 1)
        self.go([-0.48, 1.26, 1.97,-0.1,-0.48,0.01], 2)
        self.go([-0.48, 1.26, 1.97,-0.1,-0.48,0.001], 1)
        self.go([0.681,1.365, 1.22,0.655,0.68,0.001], 5) 

    
        
    
        
if __name__ == '__main__':
    rospy.init_node("command_publisher")    
    tp = SrRonexArmTargetCommander()
    tp.dum()
