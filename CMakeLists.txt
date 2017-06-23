cmake_minimum_required(VERSION 2.8.3)
project(asr_cyberglove_lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	asr_msgs
	std_msgs
	roscpp
	rospy	
)

find_package(Boost REQUIRED COMPONENTS 
	system
	program_options
)

catkin_package(
	CATKIN_DEPENDS asr_msgs std_msgs roscpp rospy
	LIBRARIES ${PROJECT_NAME}
	DEPENDS	Boost
)	

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gloveServer_impl.cc;
	src/gloveDeviceTest.cc;
	src/gloveDevice.cc;
	src/gloveStatus.cc;
	src/serial.cc 
)
# dependency to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(${PROJECT_NAME} asr_msgs_generate_messages_cpp)


add_executable(gloveServer_node src/gloveServerRun.cc)

# dependency to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(gloveServer_node asr_msgs_generate_messages_cpp)

target_link_libraries(gloveServer_node 
      ${PROJECT_NAME} 
      ${catkin_LIBRARIES} 
      ${Boost_LIBRARIES}
)

