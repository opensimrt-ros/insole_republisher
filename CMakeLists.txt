cmake_minimum_required(VERSION 3.0.2)
project(republisher)
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
add_compile_options(-std=c++17)

find_package(OpenSim 4.0 REQUIRED PATHS "${OPENSIM_HOME}"
	)

find_package(catkin REQUIRED COMPONENTS
	tf
	roscpp
	insole_msgs
	opensimrt
	)

find_package(opensimrt REQUIRED COMPONENTS
	Common
	Realtime)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp opensimrt
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	"/opt/dependencies/opensim-core/include/OpenSim/"
	"/opt/dependencies/opensim-core/include/simbody/"
	)

message(NOTICE "catkin")
foreach(dir ${catkin_INCLUDE_DIRS})
	message(STATUS
		"catkin_dir=${dir}"
		)
endforeach()

add_executable(${PROJECT_NAME}_node src/republisher_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${OpenSim_LIBRARIES}
	${opensimrt_LIBRARIES}
	)

