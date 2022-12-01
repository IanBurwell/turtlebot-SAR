# Ian's Notes:
## Setup PC
Tutorial: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Turtlebot dependancies:
```
sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-depthimage-to-laserscan \
    ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
    ros-noetic-rosserial-server ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

Turtlebot:
```
sudo apt install ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \ 
    ros-noetic-turtlebot3-gazebo
```

Default to turtlebot burger:
`echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc`

Networking:
```
export ROS_MASTER_URI=http://<HOST IP>:11311
export ROS_HOSTNAME=<HOST IP>
```

## Simulation
Tutorial: https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/

- Run basic gazebo: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
- Teleop control: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
- SLAM: `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`
- Save the map: `rosrun map_server map_saver -f ~/map`
- Navigation: `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`


# TODO:
- static transform camera transform 
- launch file include remote and rviz
- apriltags config (and print out)
- create launch file for turtlebot
- pinging issue

roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
roslaunch turtlebot3_bringup turtlebot3_remote.launch
roslaunch apriltag_ros continuous_detection.launch camera_name:="camera" image_topic:="image"
