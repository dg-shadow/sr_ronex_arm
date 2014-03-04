#!/usr/bin/env python

import rospy
from math import pi
from sr_ronex_msgs.msg import GeneralIOState, PWM


class Mapper():
    def __init__(self):
	self.pub = rospy.Publisher("/ronex/general_io/10/command/pwm/0", PWM)

    def callback(self, data):
        raw = data.analogue[6]
        effort = 200*raw/4095-100
        self.output(effort)
        

    
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/ronex/general_io/10/state", GeneralIOState, self.callback)
        rospy.spin()
        
    def output (self, effort):
        pwm_period = 6400.0
        max_effort  = 100.0
        min_effort  = -100.0
        max_pwm    = 920.0
        min_pwm    = 1000.0
#        max_pwm    =  8000.0
#        min_pwm    = 11200.0
#        max_pwm    = 8000.0
#        min_pwm    = 11200.0
#        max_pwm    = 13300.0
#        min_pwm    = 4100.0
#        max_pwm    = 15200.0
#        min_pwm    = 3100.0
#        max_pwm    = 4400.0
#        min_pwm    = 13700.0
    

        pwm_duty = ( ( effort - min_effort ) / (max_effort-min_effort)) * (max_pwm - min_pwm) + min_pwm
        
        print pwm_duty, effort
        
        pwm = PWM()
        pwm.pwm_period = pwm_period
        pwm.pwm_on_time_0 = int(pwm_duty)
        pwm.pwm_on_time_1 = 0
        print effort, pwm_duty
        self.pub.publish(pwm)
        
        
    
        
if __name__ == '__main__':
    
    m = Mapper()
    m.listener()
