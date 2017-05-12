cmake_minimum_required(VERSION 2.8.3)
project(adhoc_communication)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  fake_network
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   #AuctionStatus.msg
   ExpAuction.msg
   ExpAuctionElement.msg
   ExpFrontierElement.msg
   MmControl.msg
   MmMapUpdate.msg
   MmRobotPosition.msg
   ExpCluster.msg
   ExpClusterElement.msg
   ExpFrontier.msg
   MmListOfPoints.msg
   MmPoint.msg
   RecvString.msg
   CMgrDimensions.msg
   CMgrRobotUpdate.msg
   EmAuction.msg
   EmDockingStation.msg
   EmRobot.msg
 )



## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ChangeMCMembership.srv
   SendExpAuction.srv
   SendExpFrontier.srv
   SendExpFrontier2.srv
   SendMmMapUpdate.srv
   SendMmRobotPosition.srv
   SendQuaternion.srv
   GetNeighbors.srv
   SendExpCluster.srv
   SendMmControl.srv
   SendMmPoint.srv
   SendOccupancyGrid.srv
   SendString.srv
   GetGroupState.srv
   SendTwist.srv
   SendCMgrRobotUpdate.srv
   ShutDown.srv
   BroadcastCMgrRobotUpdate.srv
   BroadcastString.srv
   SendEmAuction.srv
   SendEmDockingStation.srv
   SendEmRobot.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#   LIBRARIES adhoc_communication
#   CATKIN_DEPENDS roscpp std_msgs
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(adhoc_communication
#   src/${PROJECT_NAME}/adhoc_communication.cpp
# )

## Declare a cpp executable

SET(MY_HEADERS
#src/McHandler.cpp
src/McHandler.h
#src/McNackFrame.cpp
src/McNackFrame.h
#src/McPosAckObj.cpp
src/McPosAckObj.h
#src/McRouteActivationFrame.cpp
src/McRouteActivationFrame.h
#src/McTree.cpp
src/McTree.h
#src/MultiHopBroadcastFrame.cpp
src/MultiHopBroadcastFrame.h
#src/Packet.cpp
src/Packet.h
#src/PositionSubscriber.cpp
src/PositionSubscriber.h
#src/RoutedFrame.cpp
src/RoutedFrame.h
#src/RouteRequest.cpp
src/RouteRequest.h
#src/RouteResponse.cpp
src/RouteResponse.h
src/structs.h
src/McDisconnectFrame.h
#src/AckLinkFrame.cpp
src/AckLinkFrame.h
#src/AckRoutedFrame.cpp
src/AckRoutedFrame.h
#src/adhoc_communication.cpp
#src/Beacon.cpp
src/Beacon.h
#src/communication_tester.cpp
#src/CrDetectionFrame.cpp
src/CrDetectionFrame.h
#src/CrSelectionFrame.cpp
src/CrSelectionFrame.h
src/DebugFunctions.h
src/defines.h
#src/EthernetFrame.cpp
src/EthernetFrame.h
src/functions.h
src/header.h
#src/Logging.cpp
src/Logging.h
#src/McAckFrame.cpp
src/McAckFrame.h
#src/McDisconnectFrame.cpp
)


add_executable(adhoc_communication src/adhoc_communication.cpp ${MY_HEADERS})
target_link_libraries(adhoc_communication ${catkin_LIBRARIES})
#add_dependencies(adhoc_communication adhoc_communication_gencpp)
add_dependencies(adhoc_communication adhoc_communication_generate_messages_cpp)
add_dependencies(adhoc_communication fake_network_generate_messages_cpp)

add_executable(communication_tester src/communication_tester.cpp)
target_link_libraries(communication_tester ${catkin_LIBRARIES})
#add_dependencies(communication_tester adhoc_communication_gencpp)
add_dependencies(communication_tester adhoc_communication_generate_messages_cpp)


#target_link_libraries(communication_tester ${catkin_LIBRARIES})
#add_dependencies(communication_tester adhoc_communication_generate_messages_cpp)
#add_dependencies(communication_tester ${rqt_marble_EXPORTED_TARGETS})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(adhoc_communication_node adhoc_communication_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(adhoc_communication
   ${catkin_LIBRARIES}
 )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS adhoc_communication adhoc_communication_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   simulation_5.launch
 #  simulation_2.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_adhoc_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

