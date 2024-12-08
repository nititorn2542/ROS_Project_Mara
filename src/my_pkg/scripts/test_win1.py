#!/usr/bin/env python3

import rospy
import tkinter as tk
import threading

class Window1:
    def __init__(self):
        rospy.init_node('window1')

        self.root = tk.Tk()
        self.root.title("ROS Window 1")

        self.button1 = tk.Button(self.root, text="Button 1", command=self.button1_callback)
        self.button1.pack(pady=10)

        self.button2 = tk.Button(self.root, text="Button 2", command=self.button2_callback)
        self.button2.pack(pady=10)

        self.button3 = tk.Button(self.root, text="Button 3", command=self.button3_callback)
        self.button3.pack(pady=10)

        self.button4 = tk.Button(self.root, text="Button 4", command=self.button4_callback)
        self.button4.pack(pady=10)

        # Start the ROS thread separately.  Critical for preventing freezes.
        self.ros_thread = threading.Thread(target=self.ros_loop)
        self.ros_thread.daemon = True  # Allow the program to exit even if the thread is running.
        self.ros_thread.start()

        self.root.mainloop()


    def button1_callback(self):
        print("Button 1 pressed")
        # Add your ROS action here (e.g., publish a message)

    def button2_callback(self):
        print("Button 2 pressed")
        # Add your ROS action here

    def button3_callback(self):
        print("Button 3 pressed")
        # Add your ROS action here

    def button4_callback(self):
        print("Button 4 pressed")
        # Add your ROS action here

    def ros_loop(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # Put your ROS logic here
            rate.sleep()


if __name__ == '__main__':
    window = Window1()
