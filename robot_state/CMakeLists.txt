cmake_minimum_required(VERSION 2.8.3)
project(robot_state)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

## Generate messages in the 'msg' folder
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

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES robot_state_lib
  CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
enable_language(CXX)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -rdynamic -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -g -O0 -fprofile-arcs -ftest-coverage -lgcov ${CMAKE_CXX_FLAGS}")

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(robot_statelib src/main.cpp)

#add_executable(robot_state_manager src/robot_state_manager.cpp)

add_library(robot_state_managerlib src/robot_state_manager.cpp)
add_dependencies(robot_state_managerlib robot_state_gencpp robot_state_generate_messages_cpp)
target_link_libraries(robot_state_managerlib ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(robot_state src/main.cpp)
add_dependencies(robot_state robot_state_gencpp robot_state_generate_messages_cpp)

#target_link_libraries(robot_statelib  ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_state robot_state_managerlib ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS robot_statelib
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

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

