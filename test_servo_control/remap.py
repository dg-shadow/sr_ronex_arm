import rospy
from math import pi
from sr_ronex_msgs.msg import GeneralIOState, PWM


class Mapper():
    def __init__(self):
	self.pub = rospy.Publisher("/ronex/general_io/10/command/pwm/0", PWM)

    def callback(self, data):
        raw = data.analogue[0]
        angle = 2*pi/4095*raw
        self.output(angle)
        

    
    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/ronex/general_io/10/state", GeneralIOState, self.callback)
        rospy.spin()
        
    def output (self, angle):
        pwm_period = 64000
        max_angle  = 2*pi
        min_angle  = 0
        max_pwm    = 12800
        min_pwm    = 6400
        
        pwm_duty = (angle/ (max_angle-min_angle)) * (max_pwm - min_pwm) + min_pwm
        
        print pwm_duty
        
        pwm = PWM()
        pwm.pwm_period = pwm_period
        pwm.pwm_on_time_0 = pwm_duty
        pwm.pwm_on_time_1 = 0
        self.pub.publish(pwm)
        
        
    
        
if __name__ == '__main__':
    
    m = Mapper()
    m.listener()
