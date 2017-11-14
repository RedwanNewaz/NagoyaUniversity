cmake_minimum_required(VERSION 2.8.3)
project(state_estimator)
## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf
  laser_geometry
)
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
set(EIGEN_INC_DIR /usr/include/eigen3/)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_estimator
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INC_DIR}
)
set(SOURCE_FILE
    src/display.cpp
    src/file_reader.cpp
    src/estimate.cpp
    src/kalman.cpp
    )
 add_executable(${PROJECT_NAME}_node src/state_estimator_node.cpp ${SOURCE_FILE})

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
 )

