import numpy as np

import time
import json
from robot_library_py import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from intera_core_msgs.msg import JointCommand
from std_msgs.msg import Bool
try:
    import thread
except ImportError:
    import _thread as thread
import argparse
from threading import Thread

class SpacemouseAsJoy:
    """
    publish same command as physical joystick. (ATTACK 3 in our lab)
    """

    def __init__(self, debug=False):  
        self.node = Node('SpacemouseAsJoy') 

        self.node.create_subscription(Joy, '/spacenav/joy', self.joy_callback, 10)
        self.pub_joy = self.node.create_publisher(Joy, '/joy', 10)
        self.pub_gripper = self.node.create_publisher(Bool, '/gripper_command', 2)
        self.pub_android = self.node.create_publisher(Bool, '/android_command', 2)
        self.pub_T = self.node.create_publisher(Bool, '/android_record', 2)
        self.joy_msg = None

        self.gripper_status=False

    def joy_callback(self, msg):
        self.joy_msg = msg 
        axes    = msg.axes
        tmp = axes[0]
        axes[0] = axes[1]
        axes[1] = tmp 

        tmp=axes[3]
        axes[3]=axes[4]
        axes[4]=tmp 

        buttons = msg.buttons
        buttons2 = [0]*11
        buttons2[0]=buttons[0]
        buttons2[1]=buttons[1]




        msgA = Bool()
        msgA.data = True                       #enable control=True
        self.pub_android.publish(msgA) 


        gripper_toggle = buttons[0]==1

        if gripper_toggle:
            self.gripper_status = not self.gripper_status


        msgB = Bool()
        msgB.data = self.gripper_status
        self.pub_gripper.publish(msgB)

        recording = buttons[1]==1
        msgT = Bool()
        msgT.data = not recording
        self.pub_T.publish(msgT) 



        cmd=Joy()
        cmd.header.stamp = rclpy.clock.Clock().now().to_msg()
        cmd.axes = axes
        cmd.buttons = buttons2

        self.pub_joy.publish(cmd)

 

if __name__=='__main__':
    rclpy.init()
    node = SpacemouseAsJoy()
    rclpy.spin(node.node)


