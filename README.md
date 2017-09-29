## loitor
A ROS version of http://github.com/loitor-vis/vi_sensor_sdk2

### Enhancements

	TODO: Ability to update camera parameters
    TODO: Make IMU work

### Publications

    /camera/left/image_raw
	/camera/right/image_raw
	/imu

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
    git clone https://github.com/billyzelsnack/loitor.git  
    cd ..  
    catkin_make  

### To run:

Launch a roscore in a new terminal if you don't already have a roscore running.

    /opt/ros/kinetic/setup.bash
    roscore

Find your the terminal you build in or open a new one.

    cd ~/loitor_ws
    source devel/setup.bash
    roscd loitor
    roslaunch launch/loitor.launch





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

