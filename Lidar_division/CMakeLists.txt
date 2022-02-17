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

add_library(filter src/Lidar_filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})
#add_dependencies(filter ${catkin_EXPORTED_TARGETS})

add_library(etc src/Lidar_etc.cpp)
target_link_libraries(etc ${catkin_LIBRARIES})
#add_dependencies(etc ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(msg src/Lidar_msg.cpp)
target_link_libraries(msg ${catkin_LIBRARIES})

add_executable(SDW_Lidar src/Lidar_code.cpp)
#target_link_libraries(SDW_Lidar ${catkin_LIBRARIES})
target_link_libraries(SDW_Lidar filter etc msg)

