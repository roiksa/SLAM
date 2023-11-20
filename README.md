Semantic SLAM for Search and Rescue Mobile Robot (Laptop side node)

Based on 
- ORBSLAM3 Ros wrapper by thien94 (https://github.com/thien94/orb_slam3_ros)
- Yolov8 ROS by Alpaca-zip (https://github.com/Alpaca-zip/ultralytics_ros)


Install libeigen3-dev
$sudo apt install libeigen3-dev

Install Pangolin
$cd ~
$git clone https://github.com/stevenlovegrove/Pangolin.git
$cd Pangolin
$mkdir build && cd build
$cmake ..
$make
$sudo make install

Install Hector Trajectory Server
$sudo apt install ros-[DISTRO]-hector-trajectory-server

Install the whole build
$ cd ~/catkin_ws/src
$ python3 -m pip install -r ultralytics_ros/requirements.txt
$ cd ~/catkin_ws
$ rosdep install -r -y -i --from-paths .
$ catkin build

