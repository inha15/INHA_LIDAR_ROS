cmake_minimum_required(VERSION 3.0.2)
project(Lidar_pkg)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   pcl_conversions
   pcl_ros
   std_msgs
   message_generation
)
add_message_files(
  FILES
  Lidar_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation roscpp std_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(SDW_Lidar src/Lidar_code.cpp)
target_link_libraries(SDW_Lidar ${catkin_LIBRARIES})


