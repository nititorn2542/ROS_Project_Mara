#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

move_cmd = Twist()

def callback_x(data):
	global move_cmd
	rospy.loginfo("X : %f",data.data)
	move_cmd.linear.x = round(data.data,2)
	
def callback_z(data):
	global move_cmd
	rospy.loginfo("Z : %f",data.data)
	move_cmd.angular.z = round(data.data,2)
	
rospy.init_node('float_to_twist')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

rospy.Subscriber('x_value',Float32,callback_x)
rospy.Subscriber('z_value',Float32,callback_z)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(move_cmd)
    rate.sleep()
