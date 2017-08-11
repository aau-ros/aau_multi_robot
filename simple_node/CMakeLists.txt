cmake_minimum_required(VERSION 2.8.3)
project(simple_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    robot_state
)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_message_files(
#  FILES
#  MyMessage.msg
#)

##add_service_files(
##    FILES
##)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime robot_state
    #  DEPENDS system_lib
)

###########
## Build ##
###########
enable_language(CXX)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -rdynamic -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -g -O0 -fprofile-arcs -ftest-coverage -lgcov ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_node src/main.cpp)
target_link_libraries(simple_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#endif()
