#!/usr/bin/env python3
import rospy
from tkinter import*
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
pub_x = rospy.Publisher('x_value', Float32, queue_size=10)
pub_z = rospy.Publisher('z_value', Float32, queue_size=10)

rospy.init_node("Window1")

#move_cmd = Twist()

def B1_click():
	x = float(E1.get())
	z = float(E2.get())
	print("Click ",x,z)
	
	pub_x.publish(x)
	pub_z.publish(z)
	
def B2_click():
	pub_x.publish(0.0)
	pub_z.publish(0.0)
	print("Stop")

frame = Tk()
frame.title("ROS Window")
frame.geometry("400x200")
frame.configure(bg="seagreen2")

L1 = Label(text="Linear X:",bg="seagreen2")
L1.place(x=50,y=25)
E1 = Entry()
E1.place(x=120,y=25)

L2 = Label(text="Angular Z:",bg="seagreen2")
L2.place(x=45,y=75)
E2 = Entry()
E2.place(x=120,y=75)

B1 = Button(text="Submit",width=5, height=1,command=B1_click)
B1.place(x=220,y=140)

B2 = Button(text="STOP",width=5, height=1,bg="red",command=B2_click)
B2.place(x=320,y=140)

frame.mainloop()
