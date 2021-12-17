cmake_minimum_required(VERSION 2.8.3)
project(dbw)
# set(ROS_BUILD_TYPE Debug)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	roscpp actionlib actionlib_msgs std_msgs sensor_msgs geometry_msgs hcpath
)
find_package(Boost REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)

add_executable(dbw src/DbwNode.cpp)
target_link_libraries(dbw ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

install(TARGETS dbw
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
