cmake_minimum_required(VERSION 2.8.3)
project(activity_recognition_ros)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg openpose_ros_msgs sensor_msgs cv_bridge image_transport)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
##########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(activity_recognition_node_sub
  include/activity_recognition_ros/human.h
  src/human.cpp
  include/activity_recognition_ros/activity_recognition.h
  src/activity_recognition.cpp
  src/activity_recognition_node_sub.cpp
)
add_dependencies(activity_recognition_node_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(activity_recognition_node_sub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)