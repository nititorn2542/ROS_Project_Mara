#!/usr/bin/env python3
import rospy
from tkinter import *
from std_msgs.msg import String

def on_change(data):
	print(data)
	#pub_pos.publish("9," + str(data))
	pub_pos.publish(f"9,{data}")

rospy.init_node('ServoWrite', anonymous=True)
pub_pinmode = rospy.Publisher('/pin_mode',String,queue_size = 10)
pub_pos = rospy.Publisher('/servo_write',String,queue_size = 10)

#rate = rospy.Rate(1)
#rate.sleep()
rospy.sleep(0.5)
pub_pinmode.publish("9,servo")
#count = 0;
#while count <= 1:
#	pub_pinmode.publish("9,servo")
#	rate.sleep()
#	count = count + 1

frame = Tk()
frame.title("Servo Write")
frame.geometry("500x300")

L1 = Label(text = "Servo Control", font = ("",30))
L1.place(x=50,y=20)

s1 = Scale(frame, from_=90, to=180, resolution=5, length=400, width=30, orient="horizontal",command = on_change)
s1.place(x=50, y=120)

frame.mainloop()
