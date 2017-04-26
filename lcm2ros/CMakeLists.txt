cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME lcm2ros)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_msgs
  roscpp
  sensor_msgs
)



find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
set (   
    lcm
    eigen3 
    bot2-lcmgl-client
    bot2-param-client
    lcmtypes_microstrain_comm
    pointcloud_math
    )

set (LINK_LIBS
    ${Boost_FILESYSTEM_LIBRARY} boost_system boost_filesystem
    bot2-core lcmtypes_kinect lcmtypes_wearnavi lcmtypes_bot2-core
    kinect-utils z jpeg-utils bot2-core
    )

# parse
add_executable(rv-lcmtoRos_segmentlist
    src/lcmtoRos_segmentlist.cpp 
)
pods_use_pkg_config_packages(rv-lcmtoRos_segmentlist lcm eigen3 bot2-lcmgl-client)    
target_link_libraries(rv-lcmtoRos_segmentlist 
    ${LINK_LIBS} ${OpenCV_LIBS} ${catkin_LIBRARIES}
    rv-cv-bridge-lcm rv-kinect-frame-pcl-utils)
pods_install_executables(rv-lcmtoRos_segmentlist)


