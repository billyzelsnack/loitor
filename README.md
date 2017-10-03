## loitor
A ROS version of http://github.com/loitor-vis/vi_sensor_sdk2

Please don't assume that this actually works at all at this point. I use this repo mostly to transfer work between my desktop and laptop.

### Enhancements

    No more settings file
    Dynamic parameters for exposure/gain
	LoitorCam and LoitorIMU messages
    TODO: Make all the parameters work
    TODO: Too much craziness
	TODO: Cleanup shutdown
    TODO: Investigate "Lost an IMU frame..."

### Publications

    /camera/left/image_raw
	/camera/right/image_raw
	/imu
    /loitor_node/parameter_descriptions
    /loitor_node/parameter_updates
	/loitor_node/get_cam
	/loitor_node/get_imu
	/loitor_node/set_cam
	/loitor_node/set_imu

### Assumptions:

* You are going to build in ~/loitor_ws  
* You have already installed and want to use ROS kinetic

### To build:

    /opt/ros/kinetic/setup.bash
    cd ~  
    mkdir loitor_ws  
    cd loitor_ws  
    mkdir src  
    catkin_make  
    source devel/setup.bash  
    cd src  
    git clone https://github.com/billyzelsnack/loitor-ros.git  
    cd ..  
    catkin_make  

### To run:

Launch a roscore in a new terminal if you don't already have a roscore running.

    /opt/ros/kinetic/setup.bash
    roscore

Find your the terminal you build in or open a new one.

    cd ~/loitor_ws
    source devel/setup.bash
    roscd loitor_ros
    rosrun loitor_ros loitor_node

To dynamically change parameters open a new terminal.

    /opt/ros/kinetic/setup.bash
    rosrun rqt_reconfigure rqt_reconfigure
    





----------------------------

# vi_sensor_sdk_v2
Loitor VI Sensor SDK V2

Before you start:
Follow this step to add loitor to your udev rules:
1. chmod +x loitor-vi-install.sh
2. sudo ./loitor-vi-install.sh
3. (maybe a restart is needed)

And... add your current user to dialout user group(Suppose your username is joeuser):
1. groups joeuser
..this will list all the groups you belong to. 
2. If you don't belong to the dialout group then add yourself to it, for example:
sudo gpasswd --add joeuser dialout
3. !!! You need to log out and log back in again for it to be effective. 

Now you can run the test program:
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./camtest

