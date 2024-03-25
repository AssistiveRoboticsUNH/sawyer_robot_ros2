
from robot_library_py import *
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from intera_core_msgs.msg import JointCommand

from threading import Thread
import time
import numpy as np 
from std_msgs.msg import Bool
import rotm2euler

import cv2
from matplotlib import pyplot as plt
import pickle
import datetime
import os 
import argparse
from sensor_msgs.msg import Image

from utils_ros import ROSInterface
import beepy as beep
import threading
# v4l2-ctl --list-devices
import imageio 

 
def main(savedir):
    rclpy.init()

    if savedir[-1]!='/':
        savedir+='/'

    if not os.path.isdir(savedir):
        os.mkdir(savedir)

    now = datetime.datetime.now()
    time_str=now.strftime("%m_%d_%Y_%H_%M")
    # fn=savedir+time_str+'_'+str(dt)+'_'+str(N) +'.pkl'
    video_path=savedir+time_str+'_rollout.mp4'
 
    video_writer = imageio.get_writer(video_path, fps=20)
    
    
    urdf_path = os.getcwd() +'/'+'sawyer.urdf'  #'/home/carl/sawyer_robot_ros2/src/teleop_script/sawyer.urdf'
    robot = URDFModel(urdf_path)
    jointMap = {name: ind for ind, name in enumerate(robot.jointNames)}

    node = Node('record_data')
    q = robot.getJoints()
    ros_interface = ROSInterface(node, robot)

    t1 = Thread(target=ros_interface.spin_thread)
    t1.start()


    msg=ros_interface.latest_joint_states

    msgs=[]
    imgs=[]
    grips=[]
    motion_detected=False
    stop_count=0

    # print('Make sure gripper is closed before starting')
    # print('record will start when gripper open for the first time.')
    print('Press T+ to start recording')
    print('Press T- to stop recording')

    started=False  #make sure gripper is closed before starting

    frequency = 20.0   # similar to robomimic

    st=time.time()
    for i in range(1000000):
        image_wrist=ros_interface.image_wrist
        image_front=ros_interface.image_front

        if image_front.sum()==0.0:
            print('image front None')
            continue
        if image_wrist.sum()==0:
            print('image wrist None')
            continue

        scale=1.0
        image_wrist = cv2.resize(image_wrist, (int(image_wrist.shape[1]*scale), int(image_wrist.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
        image_front = cv2.resize(image_front, (int(image_front.shape[1]*scale), int(image_front.shape[0]*scale) ) , interpolation = cv2.INTER_AREA)
  

        image=np.concatenate([image_wrist, image_front], axis=1) 
        bgr_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        video_writer.append_data(bgr_img)
        
        cv2.imshow('recording',image)
        if cv2.waitKey(1) == 27: 
            print('UI closed')
            break  # esc to quit

        time.sleep(1.0/frequency)

    cv2.destroyAllWindows() 


    print('done')
    dt=time.time()-st
    print( len(msgs), dt, len(msgs)/dt )

    video_writer.close()
    
    print('done')
    rclpy.shutdown()

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    # parser.add_argument("-cid", "--camera", type=int, default=0,  help="USB camera id")
    # parser.add_argument("-dir", "--savedir", type=str, default='/home/carl/data_sawyer/block/')
    parser.add_argument("-dir", "--savedir", type=str, required=True)
    args=parser.parse_args()
    print('args=', args)
    main(args.savedir)


# python3 record_video.py --savedir /home/carl/data_sawyer/spoon_pick
    