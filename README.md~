lenti_angle
===========

ROS package for reading of Lenticular Lens Displays from Fiducial Marker for angular camera position rating.

## Usage:
- roslaunch lenti_angle lenti_angle.launch

## Installation:
###Setting up a workspace for fiducial marker system with lenti_angle:

###set up ROS workspace
1. $ source /opt/ros/indigo/setup.bash
2. $ mkdir -p {workspace}/src
3. $ cd {workspace}/src
4. $ catkin_init_workspace
5. $ cd {workspace}/
6. $ catkin_make

###clone lenti_angle repository to workspace: 
1. $ cd {workspace}/src/
2. $ git clone https://github.com/eisdiele/lenti_angle.git

###clone ar_sys to {workspace}/src/
1. $ git clone https://github.com/Sahloul/ar_sys.git

###clone usb_cam to {workspace}/src/
1. $ git clone https://github.com/bosch-ros-pkg/usb_cam.git

###Copy single_board_lenti.cpp from lenti_angle/data/ to ar_sys/src/
1. $ cp {workspace}/src/lenti_angle/data/single_board_lenti.cpp {workspace}/src/ar_sys/src/single_board_lenti.cpp

###Copy board_lenti.yml from lenti_angle/data to ar_sys/data/single:
1. $ cp {workspace}/src/lenti_angle/data/board_lenti.yml {workspace}/src/ar_sys/data/single/board_lenti.yml


###Then add the following lines to {workspace}/src/ar_sys/CMakeLists:

add_executable(single_board_lenti src/single_board_lenti.cpp
	src/utils.cpp)
add_dependencies(single_board_lenti ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_board_lenti aruco ${catkin_LIBRARIES})


###Start the ROS-Package:
1. $ cd {workspace}/
2. $ catkin_make
3. $ source {workspace}/devel/setup.bash
4. $ roslaunch lenti_angle lenti_angle.launch
