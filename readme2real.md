### run ros2 docker
```
cd /home/ns/sawyer_robot_ros2/docker/sawyer-noetic
docker compose up
```


### run teleop publisher (Optional: only if you want to use joystick controller instead of android)

```
cd ~/sawyer_robot_ros2/
source install/setup.bash
ros2 run joy joy_node
```


### run teleop controller node 
* This node receive command from either android or joystick and publish as velocity commands.
```
cd ~/sawyer_robot_ros2/
source install/setup.bash
cd src/teleop_script
(if error: LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/)
python3 teleop.py

 

ros2 topic echo /robot/joint_states  (if couldn't determine topic, then run pub_dbg.py then run this command again)


```

 
### enable gripper listener using ros1 docker
```
192.168.1.57 replace with the host computer ip

sudo docker run --privileged -it --net=host pac48/sawyer_demo:latest bash

export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.46
source devel/setup.bash

rosrun intera_interface enable_robot.py -e

rosrun intera_examples gripper_keyboard.py
<esc>
 
rosrun gripper gripper_listener.py

```


### Run cameras

Realsense install: https://github.com/IntelRealSense/realsense-ros
usb_camera install: https://github.com/ros-drivers/usb_cam/tree/ros2

```
cd ~/sawyer_robot_ros2/
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/front_camera --params-file params_1.yaml


cd ~/sawyer_robot_ros2/
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=robot1 camera_name:=D455_1 rgb_camera.profile:=640x480x30
```

### Run android-as-joy

```
cd ~/sawyer_robot_ros2/
source install/setup.bash
cd /home/carl/sawyer_robot_ros2/src/teleop_script

python3 android_as_joy.py --ip 192.168.1.32
```


### record demonstration
```
cd ~/sawyer_robot_ros2/
source install/setup.bash
cd /home/carl/sawyer_robot_ros2/src/teleop_script

python3 record_data.py --savedir /home/carl/data_sawyer/dclose
```

### demos to hdf5

```
cd /home/carl/sawyer_robot_ros2/src/teleop_script
python3 demos2hdf5.py -f /home/carl/data_sawyer/dclose
```

### demos to videos

```
cd /home/carl/sawyer_robot_ros2/src/teleop_script
python3 demo2video.py -f /home/carl/data_sawyer/dclose --savedir  /home/carl/data_sawyer/videos_dclose
```


### run vscode
```
cd ~/sawyer_robot_ros2/
source install/setup.bash
code
```

