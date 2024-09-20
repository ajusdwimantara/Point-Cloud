#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import tkinter as tk
from tkinter import messagebox

class GuiPublisher:

    def __init__(self, root):
        self.root = root
        self.root.title("USER INPUT")

        # Set the size of the window (width x height)
        self.root.geometry("300x200")

        # ROS publisher setup
        self.pub = rospy.Publisher('/rotation_degree', Float32, queue_size=10)
        rospy.init_node('gui_publisher', anonymous=True)

        # Create the GUI components
        self.label = tk.Label(root, text="Enter a float value:")
        self.label.pack(pady=10)

        self.entry = tk.Entry(root)
        self.entry.pack(pady=5)

        self.button = tk.Button(root, text="Publish", command=self.publish_value)
        self.button.pack(pady=10)

    def publish_value(self):
        try:
            # Get the value from the entry widget
            value = float(self.entry.get())

            # Publish the value to the ROS topic
            msg = Float32()
            msg.data = value
            self.pub.publish(msg)

            rospy.loginfo(f"Published value: {value}")
            messagebox.showinfo("Success", f"Published value: {value}")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid float number.")

if __name__ == '__main__':
    try:
        # Create the main Tkinter window
        root = tk.Tk()
        
        # Initialize the GUI Publisher class
        gui_publisher = GuiPublisher(root)

        # Run the Tkinter main loop
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
