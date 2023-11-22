# Semantic SLAM for Search and Rescue Mobile Robot (Laptop side node)
for robot side node see (https://github.com/roiksa/SLAM-robot-)

Based on 
- ORBSLAM3 Ros wrapper by thien94 (https://github.com/thien94/orb_slam3_ros)
- Yolov8 ROS by Alpaca-zip (https://github.com/Alpaca-zip/ultralytics_ros)


## Install libeigen3-dev

```
sudo apt install libeigen3-dev
```


## Install Pangolin

```
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
make ..
make
sudo make install
```
## Install ROS NOETIC on Ubuntu 20.04

1. Add Official Noetic Repo to Ubuntu
```
echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
```

2. Add official ROS Keyring

Next, add the official ROS keyring to your Ubuntu 20.04 system. There are two ways to go about this.
The first method is to use the hkp://keyserver.ubuntu.com:80 Ubuntu key server. If this does not work, you can try to replace it with hkp://pgp.mit.edu:80. So, run the command below.

Method 1
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Method 2
```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

3.Update the ROS package index

```
sudo apt update
```

4.Install ROS Noetic on Ubuntu 20.04

```
sudo apt install ros-noetic-desktop-full
```

5.Set up ROS Noetic Environment

```
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Install Hector Trajectory Server
```
sudo apt install ros-[DISTRO]-hector-trajectory-server
```
## Install the whole build
```
cd ~/catkin_ws/src
git clone https://github.com/roiksa/SLAM.git
python3 -m pip install -r ultralytics_ros/requirements.txt
cd ~/catkin_ws
rosdep install -r -y -i --from-paths .
catkin build
```

## How to use
On laptop side
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros robot.launch
```
On the robot
```
roslaunch proto2 start.launch
```
