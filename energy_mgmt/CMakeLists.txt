cmake_minimum_required(VERSION 2.8.3)
project(energy_mgmt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  costmap_2d
  std_msgs
  message_generation
  adhoc_communication
  explorer
  map_merger
  roslib
  fake_network
  robot_state
  utilities
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  AuctionResult.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

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
  #LIBRARIES 
  CATKIN_DEPENDS roscpp rospy roslib std_msgs message_runtime adhoc_communication explorer map_merger robot_state roslib
  DEPENDS Boost
  #DEPENDS system_lib
)

###########
## Build ##
###########
enable_language(CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage -rdynamic -Wall -DBOOST_SYSTEM_NO_DEPRECATED -lboost_system -lgcov -std=c++11")

include_directories( # note: this path is relative to the CMakeLists.txt location
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(battery_simulate src/battery_simulate.cpp)
#add_library(docking src/docking.cpp)
#add_library(time_manager src/time_manager.cpp)
add_library(battery_state_updater src/battery_state_updater.cpp)
add_library(concrete_bid_computer src/concrete_bid_computer.cpp)
add_library(concrete_sender src/concrete_sender.cpp)
add_library(auction_manager src/auction_manager.cpp)
add_library(auction_observer src/auction_observer.cpp)
#add_library(docking_station_manager src/docking_station_manager.cpp)
add_library(robot_state_manager2 src/robot_state_manager2.cpp)
add_library(robot_state_manager_energy src/robot_state_manager.cpp)
#add_library(optimal_docking_station_selector src/optimal_docking_station_selector.cpp)
add_library(graph src/graph.cpp)
add_library(path_finder src/path_finder.cpp)
add_library(docking_station_detector src/docking_station_detector.cpp)

add_dependencies(battery_simulate energy_mgmt_gencpp)
#add_dependencies(docking roslib energy_mgmt_gencpp)
#add_dependencies(docking map_merger_cpp)

#add_executable(ds_mgmt src/main.cpp)
add_executable(auction_mgmt src/auction_main.cpp)
add_executable(battery_mgmt src/battery_main.cpp)

add_dependencies(robot_state_manager2 ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_state_manager_energy ${catkin_EXPORTED_TARGETS})
add_dependencies(concrete_bid_computer robot_state adhoc_communication adhoc_communication_gencpp adhoc_communication_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(concrete_sender adhoc_communication adhoc_communication_gencpp adhoc_communication_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(ds_mgmt map_merger map_merger_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(battery_mgmt map_merger map_merger_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(auction_mgmt utilities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(docking_station_detector
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(path_finder
  graph
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(auction_manager
  concrete_bid_computer
  concrete_sender
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(battery_state_updater
#  time_manager
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(battery_simulate
#  time_manager
  battery_state_updater
  robot_state_manager_energy
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#target_link_libraries(docking
#  auction_manager
#  docking_station_manager
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(ds_mgmt
#  docking
#  robot_state_manager_energy
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(battery_mgmt
  #  battery
  battery_simulate
  robot_state_manager_energy
#  time_manager
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(auction_observer
  auction_manager
  robot_state_manager_energy
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(auction_mgmt
  auction_manager
  auction_observer
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

#    add_library(mock_time_manager test/mock_time_manager.cpp)  
    add_executable(mock_robot test/mock_robot.cpp)
    target_link_libraries(mock_robot ${catkin_LIBRARIES})
    add_dependencies(mock_robot
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_library(mock_docking_station_manager test/mock_docking_station_manager.cpp)
    target_link_libraries(mock_docking_station_manager ${catkin_LIBRARIES})
    add_dependencies(mock_docking_station_manager
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_library(mock_distance_computer test/mock_distance_computer.cpp)
    target_link_libraries(mock_distance_computer ${catkin_LIBRARIES})
    add_dependencies(mock_distance_computer
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )


    #TODO use sub-cmakelists files
    add_rostest_gtest(battery_state_updater_utest
        test/battery_state_updater_utest.launch # apparently the launch file must be the first, or won't compile
        test/battery_state_updater_utest.cpp        
    )
    target_link_libraries(battery_state_updater_utest
        battery_state_updater
        robot_state_manager2
        ${catkin_LIBRARIES}
    )
    add_dependencies(battery_state_updater_utest
        explorer_generate_messages_cpp
        explorer_gencpp #TODO do we need both gencpp and generate_*?
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(auction_manager_utest
        test/auction_manager_utest.launch
        test/auction_manager_utest.cpp              
    )
    add_library(mock_bid_computer test/mock_bid_computer.cpp)
    add_library(mock_sender test/mock_sender.cpp)
    target_link_libraries(auction_manager_utest
        auction_manager
        mock_bid_computer
        mock_sender
        ${catkin_LIBRARIES}
    )
    add_dependencies(mock_sender adhoc_communication adhoc_communication_gencpp adhoc_communication_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
    add_dependencies(auction_manager_utest
        adhoc_communication_generate_messages_cpp
        adhoc_communication_gencpp #TODO do we need both gencpp and generate_*?
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(auction_observer_utest
        test/auction_observer_utest.launch
        test/auction_observer_utest.cpp              
    )
    add_library(mock_auction_manager test/mock_auction_manager.cpp)
    target_link_libraries(auction_observer_utest
        auction_observer
        mock_auction_manager
        robot_state_manager2
        ${catkin_LIBRARIES}
    )
    add_dependencies(mock_auction_manager
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )
    add_dependencies(auction_observer_utest
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(concrete_bid_computer_utest
        test/concrete_bid_computer_utest.launch
        test/concrete_bid_computer_utest.cpp        
    )
    target_link_libraries(concrete_bid_computer_utest
        concrete_bid_computer
        ${catkin_LIBRARIES}
    )
    add_dependencies(concrete_bid_computer_utest
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(graph_utest
        test/graph_utest.launch
        test/graph_utest.cpp        
    )
    target_link_libraries(graph_utest
        graph
        ${catkin_LIBRARIES}
    )
    add_dependencies(graph_utest
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(path_finder_utest
        test/path_finder_utest.launch
        test/path_finder_utest.cpp        
    )
    target_link_libraries(path_finder_utest
        path_finder
        graph
        ${catkin_LIBRARIES}
    )
    add_dependencies(path_finder_utest
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(docking_station_detector_utest
        test/docking_station_detector_utest.launch
        test/docking_station_detector_utest.cpp        
    )
    target_link_libraries(docking_station_detector_utest
        docking_station_detector
        mock_docking_station_manager
        mock_distance_computer
        ${catkin_LIBRARIES}
    )
    add_dependencies(docking_station_detector_utest
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )

    add_rostest_gtest(missing_utests
        test/missing_utests.launch
        test/missing_utests.cpp        
    )
    target_link_libraries(missing_utests
        robot_state_manager_energy
        battery_simulate
#        docking
        ${catkin_LIBRARIES}
    )
    add_dependencies(missing_utests
        ${PROJECT_NAME}_generate_messages_cpp
        ${catkin_EXPORTED_TARGETS}
    )
#    add_rostest_gtest(itest
#        test/itest.launch
#        test/itest.cpp              
#    )
#    target_link_libraries(itest
#        ${catkin_LIBRARIES}
#    )

#  add_rostest_gtest(utest
#  test/test.launch
#  test/utest.cpp
#  )
#  target_link_libraries(utest
#  battery_simulate
#  mock_time_manager
#  ${catkin_LIBRARIES}
#  )
#  add_dependencies(utest
#  adhoc_communication_generate_messages_cpp
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${catkin_EXPORTED_TARGETS}
#  )


#  add_rostest_gtest(utest_docking
#  test/test_docking.launch
#  test/utest_docking.cpp
#  )
#  target_link_libraries(utest_docking
#  docking
#  #mock_time_manager
#  ${catkin_LIBRARIES}
#  )
#  add_dependencies(utest_docking
#  adhoc_communication_generate_messages_cpp
#  ${PROJECT_NAME}_generate_messages_cpp
#  ${catkin_EXPORTED_TARGETS}
#  )
endif()
