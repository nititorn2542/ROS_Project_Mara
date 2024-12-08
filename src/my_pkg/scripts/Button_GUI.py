#!/usr/bin/env python3
import rospy
import time
from tkinter import *
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist

move_cmd = Twist()
start = False

def callback(data):
    global start, move_cmd  # Declare start and move_cmd as global

    if data.data == 1.0:
        frame.config(bg="yellow")
        if start == False:
            move_cmd.linear.x = 2.0
            move_cmd.angular.z = 2.0
        else:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        start = not start
    else:
        frame.config(bg="black")


def publish_velocity():
    pub_turtle.publish(move_cmd)
    frame.after(100, publish_velocity) # reschedule publish_velocity after 100ms


rospy.init_node('Button_GUI')
pub = rospy.Publisher('/pin_mode',String,queue_size = 10)
pub_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(1)
rate.sleep()
pub.publish("3,input")
	
rospy.Subscriber('/sensor_3', Float32, callback)

frame = Tk()
frame.title("ROS Sub Window")
frame.geometry("400x200")
frame.config(bg="slateblue2")

publish_velocity() # Start publishing velocity commands

frame.mainloop()
