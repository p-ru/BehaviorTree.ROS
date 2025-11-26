cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(behaviortree_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp 
    behaviortree_cpp_v3
    actionlib

    std_msgs
    actionlib_msgs
    geometry_msgs
    nav_msgs
    dji_srvs
    apriltag_msgs
    
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

add_service_files(
   FILES
   AddTwoInts.srv
 )

add_action_files(
   DIRECTORY action
   FILES     Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS message_runtime 
    ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

######################################################
# LIBRARIES

#header only for the time being

#add_library(behaviortree_ros ... )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

######################################################
# TESTS

add_executable(test_bt test/test_bt.cpp)
add_dependencies(test_bt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bt ${catkin_LIBRARIES} )

add_executable(test_server test/test_server.cpp)
add_dependencies(test_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_server ${catkin_LIBRARIES} )


add_executable(waypoint_bt_node src/waypoint_bt_node.cpp)
add_dependencies(waypoint_bt_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_bt_node ${catkin_LIBRARIES} )

add_executable(trajgen_supervisor src/trajgen_supervisor.cpp)
add_dependencies(trajgen_supervisor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajgen_supervisor ${catkin_LIBRARIES} )



catkin_install_python(PROGRAMS
  scripts/record_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


######################################################
# INSTALL

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

######################################################
# EXAMPLES and TOOLS



