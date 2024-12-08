#!/usr/bin/env python3
import rospy
from tkinter import*
from std_msgs.msg import Float32

x_value = 0
z_value = 0

def callback_x(data):
	rospy.loginfo("X : %f",data.data)
	L3.config(text=str(round(data.data,2)) + " m/s")
	
def callback_z(data):
	rospy.loginfo("Z : %f",data.data)
	L4.config(text=str(round(data.data,2)) + " rad/s")
	
rospy.init_node('Window2')
rospy.Subscriber('x_value',Float32,callback_x)
rospy.Subscriber('z_value',Float32,callback_z)

frame = Tk()
frame.title("ROS Sub Window")
frame.geometry("400x200")
frame.configure(bg="slateblue2")

L1 = Label(text="X value : ",bg="slateblue2",font=('Arial',20))
L1.place(x=80,y=50)
L2 = Label(text="Z value : ",bg="slateblue2",font=('Arial',20))
L2.place(x=80,y=115)
L3 = Label(text="0 m/s",bg="slateblue2",font=('Arial',20))
L3.place(x=220,y=50)
L4 = Label(text="0 rad/s",bg="slateblue2",font=('Arial',20))
L4.place(x=220,y=115)

frame.mainloop()
