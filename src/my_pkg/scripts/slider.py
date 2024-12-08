#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tkinter import *

def update_velocity():
    # Retrieve velocity from the scales
    twist = Twist()
    twist.linear.x = s1.get()
    twist.angular.z = s2.get()

    # Publish velocity to the `/turtle1/cmd_vel` topic
    pub.publish(twist)
    
    # Update the function again after 100 ms
    frame.after(100, update_velocity)

# Initialize the ROS node
rospy.init_node('Slider_GUI', anonymous=True)

# Create publisher for controlling the turtle
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

# Configure the Tkinter GUI
frame = Tk()
frame.title("Slider GUI")
frame.geometry("500x400")

# Create the scales (sliders)
s1 = Scale(frame, label="speedX", from_=1, to=-1, resolution=0.1, length=200, width=30, orient="vertical")
s1.place(x=70, y=100)

s2 = Scale(frame, label="speedZ", from_=-1, to=1, resolution=0.1, length=150, width=30, orient="horizontal")
s2.place(x=250, y=150)

# Setup the update loop in the GUI
update_velocity()

# Start the Tkinter main loop
frame.mainloop()

