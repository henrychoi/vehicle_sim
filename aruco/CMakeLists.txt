cmake_minimum_required(VERSION 2.8.3)
project(aruco)

add_compile_options(-std=c++17)

#Packages
find_package(catkin REQUIRED COMPONENTS	
	tf cv_bridge roscpp std_msgs image_transport
)
find_package(OpenCV REQUIRED)

#Catkin dependencies
catkin_package(CATKIN_DEPENDS tf roscpp std_msgs)

add_executable(aruco src/ArucoNode.cpp)
target_link_libraries(aruco ${catkin_LIBRARIES} ${OpenCV_LIBS})

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

install(TARGETS aruco
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
