cmake_minimum_required(VERSION 2.8.3)
project(clutter_butter)
add_compile_options(-std=c++11 -Wall -Wextra)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(
	FILES
	Target.msg
	PushPlan.msg
)

add_service_files(
	FILES
	SetMapWalkerState.srv
	SetPushExecutorState.srv
)

## Needs to be declared be for catkin_package
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  include
#  LIBRARIES
  CATKIN_DEPENDS
  std_msgs
  message_runtime
)

###########
## Build ##
###########


include_directories(
	include ${catkin_INCLUDE_DIRS} 
	${GTEST_INCLUDE_DIRS}
)
 
add_executable(
	walker 
	src/walker.cpp
)
target_link_libraries(
	walker
	${catkin_LIBRARIES}
)
add_dependencies(
	walker
	${PROJECT_NAME}_generate_messages_cpp
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    walker_tests
    test/utest.launch
    test/utest.cpp
  )
  target_link_libraries(
    walker_tests
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    walker_tests
    walker
    ${PROJECT_NAME}_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
  )
endif()
