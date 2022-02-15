cmake_minimum_required(VERSION 2.8.3)
project(hcpath)
# set(ROS_BUILD_TYPE Debug)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
	roscpp std_msgs actionlib actionlib_msgs message_generation
	geometry_msgs nav_msgs visualization_msgs gazebo_msgs
	roslib costmap_2d tf
	)
#find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

add_message_files(FILES PathState.msg PathState.msg GaussianPathState.msg)
#add_service_files(FILES path_srv.srv)
add_action_files(FILES park.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	CATKIN_DEPENDS roscpp std_msgs message_runtime actionlib_msgs gazebo_msgs
  	DEPENDS EIGEN3 gazebo_ros
  )
catkin_package(DEPENDS Boost)

add_executable(hcpath src/HcPathNode.cpp src/cute_c2.cpp
	src/configuration.cpp  src/utilities.cpp src/ekf.cpp
	src/hc_cc_circle.cpp src/paths.cpp
	src/hc_cc_state_space.cpp
	src/hc_cc_state_space.cpp
	src/hcpmpm_reeds_shepp_state_space.cpp
	src/cc_dubins_state_space.cpp
	src/cc00_dubins_state_space.cpp  src/cc0pm_dubins_state_space.cpp 
	src/ccpm0_dubins_state_space.cpp src/ccpmpm_dubins_state_space.cpp
	)
target_link_libraries(hcpath ${catkin_LIBRARIES}
# 	${Boost_INCLUDE_DIRS} ${GAZEBO_LIBRARY_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

install(TARGETS hcpath
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_dependencies(hcpath ${hcpath_EXPORTED_TARGETS})

