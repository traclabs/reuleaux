cmake_minimum_required(VERSION 2.8.3)
project(reachability)


find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  message_generation
  geometry_msgs
  std_msgs
  tf2
  pcl_ros
  trac_ik_lib
  kdl_parser
  urdf
  reachability_msgs
)

find_package(octomap REQUIRED)
find_package(PkgConfig)
pkg_check_modules(HDF5 REQUIRED hdf5)


catkin_package(

  INCLUDE_DIRS
    include
  LIBRARIES
    sphere_discretization
    robot_kinematics
    hdf5_dataset
    CATKIN_DEPENDS
    trac_ik_lib
    message_runtime
    kdl_parser
    urdf
    reachability_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${HDF5_INCLUDE_DIR}) 

add_library(sphere_discretization  src/sphere_discretization.cpp)
target_link_libraries(sphere_discretization  ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARY_DIRS})

add_library(robot_kinematics  src/robot_kinematics.cpp)
target_link_libraries(robot_kinematics  ${catkin_LIBRARIES} -llapack)

add_library(hdf5_dataset  src/hdf5_dataset.cpp)
target_link_libraries(hdf5_dataset  ${catkin_LIBRARIES} ${HDF5_LINK_LIBRARIES})


add_executable(create_reachability_map src/create_reachability_map.cpp)
target_link_libraries(create_reachability_map sphere_discretization robot_kinematics hdf5_dataset ${catkin_LIBRARIES} )

add_executable(create_inverse_reachability_map src/create_inverse_reachability_map.cpp)
target_link_libraries(create_inverse_reachability_map sphere_discretization robot_kinematics  hdf5_dataset ${catkin_LIBRARIES})

add_executable(debug_visualize_poses src/debug_visualize_poses.cpp)
target_link_libraries(debug_visualize_poses sphere_discretization ${catkin_LIBRARIES} )


add_executable(load_reachability_map src/load_reachability_map.cpp)
target_link_libraries(load_reachability_map  hdf5_dataset ${catkin_LIBRARIES})


#add_executable(create_capability_map src/create_capability_map.cpp)
#add_dependencies(create_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(create_capability_map sphere_discretization kinematics hdf5_dataset ${catkin_LIBRARIES})


#add_executable(load_capability_map src/load_capability_map.cpp)
#add_dependencies(load_capability_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(load_capability_map  hdf5_dataset ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(utest test/utest.cpp)
#  target_link_libraries(utest sphere_discretization ${catkin_LIBRARIES})
#endif()

install(TARGETS sphere_discretization hdf5_dataset robot_kinematics
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
