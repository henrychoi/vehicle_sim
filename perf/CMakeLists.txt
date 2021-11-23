cmake_minimum_required(VERSION 2.8.3)
project(perf)
# set(ROS_BUILD_TYPE Debug)
add_compile_options(-std=c++17)

#Packages
find_package(catkin REQUIRED COMPONENTS	
	tf tf2 tf2_ros roscpp std_msgs gazebo_msgs
)

catkin_package(CATKIN_DEPENDS tf2 tf2_ros roscpp std_msgs gazebo_msgs)

add_executable(perf src/PerfNode.cpp)
target_link_libraries(perf ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})

install(TARGETS perf
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
