cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_BUILD_TYPE RELEASE)
project(rosbag_decoder)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  hector_mapping
  pointcloud_to_laserscan
  velodyne_driver
  velodyne_height_map

)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosbag_decoder
  CATKIN_DEPENDS hector_mapping pointcloud_to_laserscan velodyne_driver velodyne_height_map
#  DEPENDS system_lib
)
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

# ---------------------------- GUI --------------------------------------------------
set(QT_LIBRARIES Qt5::Core)
set (PROJECT_LIB
    ${catkin_LIBRARIES}
   ${Qt5Core_LIBRARIES}


)
set(GUI_SOURCE_FILES
  src/rosbag_decoder_node.cpp
  src/mainwindow.cpp
  src/filereader.cpp
)

set(GUI_HEADER_FILES
  src/mainwindow.h
  src/filereader.h
)

# *.ui
set(GUI_UI_FILES
  src/mainwindow.ui
)

# *.qrc
set(GUI_RESOURCE_FILES
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_AUTOUIC ON)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
qt5_wrap_cpp(QT_MOC_HPP ${GUI_HEADER_FILES})
include_directories(${Qt5Widgets_INCLUDE_DIRS})



add_executable(gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP}
                                  ${QT_RESOURCES_CPP}
                                  ${QT_MOC_HPP})

target_link_libraries(gui ${catkin_LIBRARIES} Qt5::Widgets)




