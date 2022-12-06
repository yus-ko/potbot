#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import sys
import io
from ctypes import Structure, sizeof, c_long, c_int32, c_uint16
import math
from geometry_msgs.msg import Twist
import glob
import os

import Tkinter as tk

class Application(tk.Frame,object):
    def __init__(self, master = None):
        super(Application, self).__init__(master)

        self.master.title("Scale")     

        #---------------------------------------------------------------
        

        
        #scaleV = tk.Scale( self.master)
        #scaleV.pack(side = tk.RIGHT)

        
        self.scale_var = tk.DoubleVar()
        scaleH = tk.Scale( self.master, 
                    variable = self.scale_var, 
                    command = self.slider_scroll,
                    orient=tk.HORIZONTAL,   
                    length = 300,           
                    width = 20,             
                    sliderlength = 20,      
                    from_ = -math.pi,            
                    to = math.pi,               
                    resolution=0.01,         
                    tickinterval=0         
                    )
        scaleH.pack()
        #---------------------------------------------------------------

        #self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size =20)
        self.pub_coe = rospy.Publisher('/potential_coefficient', Float32, queue_size =20)
        rospy.init_node('gui_control', anonymous=True)

    def slider_scroll(self, event=None):
        
        print(str(self.scale_var.get()))
        # cmd = Twist()
        # cmd.linear.x = self.scale_var.get()
        # self.pub_cmd.publish(cmd)
        coe = Float32()
        coe.data = self.scale_var.get()
        self.pub_coe.publish(coe)

if __name__ == "__main__":
    root = tk.Tk()
    app = Application(master = root)
    app.mainloop()

# if __name__ == '__main__':

#     pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size =20)
#     rospy.init_node('keyboard_input', anonymous=True)
            
#     pub_cmd.publish(cmd)