lenti_angle
===========

ROS package for reading of Lenticular Lens Displays from Fiducial Marker for angular camera position rating.

## Usage:
- roslaunch lenti_angle lenti_angle.launch

## Installation:
#Setting up a workspace for fiducial marker system with lenti_angle:

#set up ROS workspace
$ source /opt/ros/indigo/setup.bash
$ mkdir -p {workspace}/src
$ cd {workspace}/src
$ catkin_init_workspace
$ cd {workspace}/
$ catkin_make

#clone lenti_angle repository to workspace: 
$ cd {workspace}/src/
$ git clone https://github.com/eisdiele/lenti_angle.git

#clone ar_sys to {workspace}/src/
$ git clone https://github.com/Sahloul/ar_sys.git

#clone usb_cam to {workspace}/src/
$ git clone https://github.com/bosch-ros-pkg/usb_cam.git

#Copy single_board_lenti.cpp from lenti_angle/data/ to ar_sys/src/
$ cp {workspace}/src/lenti_angle/data/single_board_lenti.cpp {workspace}/src/ar_sys/src/single_board_lenti.cpp

#Copy board_lenti.yml from lenti_angle/data to ar_sys/data/single:
$ cp {workspace}/src/lenti_angle/data/board_lenti.yml {workspace}/src/ar_sys/data/single/board_lenti.yml


#Then add the following lines to {workspace}/src/ar_sys/CMakeLists:

add_executable(single_board_lenti src/single_board_lenti.cpp
	src/utils.cpp)
add_dependencies(single_board_lenti ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_board_lenti aruco ${catkin_LIBRARIES})


#Start the ROS-Package:
$ cd {workspace}/
$ catkin_make
$ source {workspace}/devel/setup.bash
$ roslaunch lenti_angle lenti_angle.launch
