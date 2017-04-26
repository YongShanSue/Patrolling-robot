cmake_minimum_required(VERSION 2.6.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME kinect-frame-pcl-kobe-demo)
include(cmake/pods.cmake)

find_package(PCL 1.7 REQUIRED COMPONENTS common io features) # visualization
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

set (CONFIG_PACKAGES     
    lcm
    eigen3 
    bot2-lcmgl-client
    bot2-param-client
    lcmtypes_microstrain_comm
    pointcloud_math
    )

set (LINK_LIBS
    ${Boost_FILESYSTEM_LIBRARY} boost_system boost_filesystem
    ${PCL_COMMON_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_FEATURES_LIBRARIES}  ${PCL_VISUALIZATION_LIBRARIES} ${PCL_LIBRARIES}
    bot2-core lcmtypes_kinect lcmtypes_wearnavi lcmtypes_bot2-core
    kinect-utils z jpeg-utils bot2-core
    ) 

add_library(rv-kinect-frame-pcl-utils SHARED src/rv-kinect-frame-pcl-utils.cpp)
target_link_libraries(rv-kinect-frame-pcl-utils ${LINK_LIBS} ${OpenCV_LIBS})
set_target_properties(rv-kinect-frame-pcl-utils PROPERTIES SOVERSION 1)
pods_install_libraries(rv-kinect-frame-pcl-utils)
pods_install_headers(src/rv-kinect-frame-pcl-utils.h DESTINATION kinect)
pods_install_pkg_config_file(rv-kinect-frame-pcl-utils
    LIBS -lkinect-frame-pcl-utils
    REQUIRES lcm eigen3
    VERSION 0.0.1)

# parse
add_executable(rv-kinect-frame-pcl-kobe-demo
    src/rv-kinect-frame-pcl-lcm-simple-kobe.cpp 
    src/rv-cv-pcl-utils.cpp
    src/haptic-array-generator.cpp)
pods_use_pkg_config_packages(rv-kinect-frame-pcl-kobe-demo lcm eigen3 bot2-lcmgl-client)    
target_link_libraries(rv-kinect-frame-pcl-kobe-demo
    ${LINK_LIBS} ${OpenCV_LIBS} 
    rv-cv-bridge-lcm rv-kinect-frame-pcl-utils)
pods_install_executables(rv-kinect-frame-pcl-kobe-demo)

# vis
add_executable(rv-vis
    src/rv-vis.cpp src/rv-cv-pcl-utils.cpp)
pods_use_pkg_config_packages(rv-vis 
    lcm eigen3 bot2-lcmgl-client )    
target_link_libraries(rv-vis 
    ${LINK_LIBS} ${OpenCV_LIBS} 
    rv-cv-bridge-lcm rv-kinect-frame-pcl-utils bot2-lcmgl-client
    )
pods_install_executables(rv-vis)

# caffe
#add_subdirectory(src)

