cmake_minimum_required(VERSION 2.8.3)
project(robot_state)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    data_logger
)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_message_files(
#  FILES
#  MyMessage.msg
#)

add_service_files(
    FILES
    GetRobotState.srv
    SetRobotState.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES robot_state_management
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime data_logger
    #  DEPENDS system_lib
)

###########
## Build ##
###########
enable_language(CXX)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -rdynamic -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -g -O0 -fprofile-arcs -ftest-coverage -lgcov ${CMAKE_CXX_FLAGS}")

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(server src/server.cpp)
add_dependencies(server robot_state_gencpp robot_state_generate_messages_cpp)
target_link_libraries(server ${catkin_LIBRARIES})

add_executable(robot_state src/main.cpp)
add_dependencies(robot_state robot_state_gencpp robot_state_generate_messages_cpp)
target_link_libraries(robot_state server ${catkin_LIBRARIES})

add_library(robot_state_management src/robot_state_interface.cpp)
target_link_libraries(robot_state_management ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS robot_state_management
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#    PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(robot_state_test
        test/test.launch
        test/utest.cpp
    )
    target_link_libraries(robot_state_test
        ${catkin_LIBRARIES}
    )
    add_dependencies(robot_state_test
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

endif()
