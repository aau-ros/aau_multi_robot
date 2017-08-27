cmake_minimum_required(VERSION 2.8.3)
project(utilities)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES data_logger time_manager message_logger
    CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
#    DEPENDS system_lib
)


###########
## Build ##
###########
enable_language(CXX)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -rdynamic -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -g -O0 -fprofile-arcs -ftest-coverage -lgcov ${CMAKE_CXX_FLAGS}")

include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(data_logger src/data_logger.cpp)
add_dependencies(data_logger utilities_gencpp utilities_generate_messages_cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})

add_library(time_manager src/time_manager.cpp src/mock_time_manager.cpp)
add_dependencies(time_manager utilities_gencpp utilities_generate_messages_cpp)
target_link_libraries(time_manager ${catkin_LIBRARIES})

add_library(message_logger src/message_logger_concrete.cpp src/message_logger_mock.cpp)
add_dependencies(message_logger utilities_gencpp utilities_generate_messages_cpp)
target_link_libraries(message_logger ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS data_logger
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS time_manager message_logger
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
endif()

