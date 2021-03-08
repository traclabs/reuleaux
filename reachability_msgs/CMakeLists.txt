cmake_minimum_required(VERSION 2.8.3)
project(reachability_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  sensor_msgs
  geometry_msgs
  octomap_msgs
  nav_msgs
  visualization_msgs
  trajectory_msgs
  message_generation
  genmsg
  )


## Generate messages in the 'msg' folder
add_message_files(
  FILES
 capShape.msg
 capability.msg
 WsSphere.msg
 WorkSpace.msg
  )

## Generate services in the 'srv' folder
add_service_files(
  FILES
#  StartController.srv
)


## Generate actions in the 'action' folder
#add_action_files(
#  DIRECTORY action
#  FILES
#  JointStateTrajectoryExecutor.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  octomap_msgs
  control_msgs
  nav_msgs
  trajectory_msgs
#  actionlib
  visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS message_runtime control_msgs geometry_msgs octomap_msgs sensor_msgs nav_msgs trajectory_msgs visualization_msgs # roscpp rospy actionlib
   DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
