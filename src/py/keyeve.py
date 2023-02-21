#!/usr/bin/env python
import sys
import select
import termios
import tty
import rospy
from geometry_msgs.msg import Twist

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_key_status():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                break
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key, True if key in ["q", "Q"] else False

if __name__ == "__main__":

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size =20)
    rospy.init_node('keyboard_input', anonymous=True)

    vel_straight = 0.1
    vel_back = -0.1
    vel_straight_sprint = 0.4
    vel_back_sprint = -0.4

    while not rospy.is_shutdown():
        key = getch()
        # key, key_status = get_key_status()
        # print(key, key_status)
        cmd = Twist()
        if key == "w":
            cmd.linear.x = vel_straight_sprint
        elif key == "s":
            cmd.linear.x = vel_back_sprint
        elif key == "c":
            break
        pub_cmd.publish(cmd)
