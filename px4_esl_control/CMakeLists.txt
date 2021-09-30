cmake_minimum_required(VERSION 3.0.2)
project(px4_esl_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES px4_esl_control
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/vector3_to_setpoint.cpp
)

# add_executable(offb_node src/offb_node.cpp)
# add_executable(pos_control_node src/pos_control_node.cpp)
add_executable(vector3_to_setpoint src/vector3_to_setpoint.cpp)

# target_link_libraries(offb_node
#    ${catkin_LIBRARIES} ${mavros_LIBRARIES} ${mavlink_LIBRARIES}
# )
# target_link_libraries(pos_control_node
#    ${catkin_LIBRARIES} ${mavros_LIBRARIES} ${mavlink_LIBRARIES}
# )
target_link_libraries(vector3_to_setpoint
   ${catkin_LIBRARIES} ${mavros_LIBRARIES} ${mavlink_LIBRARIES}
)
