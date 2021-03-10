cmake_minimum_required(VERSION 2.8.3)
project(reachability_stance_generators)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  craftsman_msgs
  stance_generator_base
  planner_interface
  planner_viz
  reachability
  reachability_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES reachability_stance_generators
 CATKIN_DEPENDS roscpp stance_generator_base geometry_msgs craftsman_msgs planner_interface planner_viz reachability reachability_msgs
 DEPENDS )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(reachability_stance_generators
  src/reachability_stance_generator.cpp
)

target_link_libraries(reachability_stance_generators
  ${catkin_LIBRARIES}
  )

## Mark executables and/or libraries for installation
 install(TARGETS reachability_stance_generators
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install( FILES reachability_stance_generators.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach( dir launch )
   install( DIRECTORY ${dir}/
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir} )
endforeach(dir)
