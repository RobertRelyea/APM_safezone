cmake_minimum_required(VERSION 2.8.3)


cmake_minimum_required(VERSION 2.8)
project(safezone)

# External Packages
find_package(OpenCV REQUIRED)
find_package(Caffe REQUIRED)

find_package(PCL 1.7 REQUIRED REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ROS Packages 
find_package(catkin REQUIRED roscpp std_msgs sensor_msgs message_generation  pcl_ros pcl_conversions)

# ROS Messages
generate_messages(
  DEPENDENCIES
  sensor_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(safezone main.cpp classifier.cpp ipm.cpp)

## Add cmake target dependencies of the executable
add_dependencies(safezone ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(safezone
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


include_directories(${OpenCV_INCLUDE_DIRS} ${Caffe_INCLUDE_DIRS})
target_link_libraries(safezone ${OpenCV_LIBS} ${Caffe_LIBRARIES})
catkin_package()