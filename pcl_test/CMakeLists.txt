cmake_minimum_required(VERSION 3.0.2)
project(pcl_test)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
)

#set(OpenCV_INCLUDE_DIRS "/usr/local/include")
#set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.8")
#set(PCL_LIBRARY_DIRS "/usr/local/lib")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node 
                    src/pcl_test_node.cpp
                    src/pcl_test_core.cpp)


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

