#!/usr/bin/env python3
import rospy
from tkinter import*
from std_msgs.msg import String

rospy.init_node("ONOFF_GUI")
pub_pinmode = rospy.Publisher('pin_mode', String, queue_size=10)
pub = rospy.Publisher('digital_write', String, queue_size=10)

status = False

def B1_click():
	global status
	print("ON")
	pub.publish("13,1")
	B3.config(text="OFF",bg="yellow")
	status = True
	
def B2_click():
	global status
	print("OFF")
	pub.publish("13,0")
	B3.config(text="ON",bg="orange")
	status = False
	
def B3_click():
	global status
	if status == True:
		print("one push : OFF")
		pub.publish("13,0")
		B3.config(text="ON",bg="orange")
	else:
		print("one push : ON")
		pub.publish("13,1")
		B3.config(text="OFF",bg="yellow")
	status = not status

frame = Tk()
frame.title("ON - OFF Window")
frame.geometry("200x300")
frame.configure(bg="lightblue")

B1 = Button(text="ON",width=5, height=1,command=B1_click,bg="green",fg="white")
B1.pack(padx=20,pady=20,fill="both",expand=True)

B2 = Button(text="OFF",width=5, height=1,command=B2_click,bg="red",fg="white")
B2.pack(padx=20,pady=20,fill="both",expand=True)

B3 = Button(text="ON",width=5, height=1,command=B3_click,bg="orange",fg="black")
B3.pack(padx=20,pady=20,fill="both",expand=True)

pub_pinmode.publish("13,output")
pub.publish("13,0")

frame.mainloop()
