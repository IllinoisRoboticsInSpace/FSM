#!/usr/bin/env python
import roslib
roslib.load_manifest('finite_state_machine')
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_listener():
    rospy.init_node('joy_simulator', anonymous = True)
    rospy.Subscriber('cmd_vel', Twist, joy_publisher)
    rospy.spin()

def joy_publisher(data):
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    joy_msg = Joy() #make a new joy message
    joy_msg.header.stamp = rospy.Time.now()
    for i in range(8):
        joy_msg.axes+= [0]
    for i in range(11):
        joy_msg.buttons += [0]
    #based on this - axes: [turn_vel, fwd_vel,...,bin_command(+1 means extend, -1 means retract)]
    joy_msg.axes[0] = angular_vel
    joy_msg.axes[1] = linear_vel

    #get_param
    param = rospy.get_param("/bin/command")
    if param < -0.01  and param>-1:
        joy_msg.axes[7] = -1
    elif param>0.01 and param<1:
        joy_msg.axes[7] =  1
    else:
        joy_msg.axes[7] =  0

    #get the turbine param
    turbine_param = rospy.get_param("/bin/turbine")
    if (turbine_param):
        joy_msg.buttons[5] = 1
        joy_msg.buttons[4] = 0
    if (turbine_param == 0):
        joy_msg.buttons[4] = 1
        joy_msg.buttons[5] = 0

    pub = rospy.Publisher('joy2', Joy)
    pub.publish(joy_msg)

    

if __name__ == '__main__':
    joy_listener()
    
        
