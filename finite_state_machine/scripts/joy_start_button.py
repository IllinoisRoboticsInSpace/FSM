#!/usr/bin/env python
import roslib
roslib.load_manifest('finite_state_machine')
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def joy_listener():
    rospy.init_node('publish_start', anonymous = True)
    rospy.Subscriber('state_machine_trigger', Bool, joy_publisher)
    rospy.spin()

def joy_publisher(data):
    print 'bkdaljfasd'
    joy_msg = Joy() #make a new joy message
    joy_msg.header.stamp = rospy.Time.now()
    for i in range(8):
        joy_msg.axes+= [0]
    for i in range(11):
        joy_msg.buttons += [0]
    #based on this - axes: [turn_vel, fwd_vel,...,bin_command(+1 means extend, -1 means retract)]
    joy_msg.buttons[7] = 1

    pub = rospy.Publisher('joy', Joy)
    pub.publish(joy_msg)

    

if __name__ == '__main__':
    joy_listener()
    
        
