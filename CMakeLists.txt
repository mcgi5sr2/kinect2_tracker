cmake_minimum_required(VERSION 2.8.3)
project(kinect2_tracker)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  kdl_conversions
  roscpp
  roslib
  tf
  tf_conversions
  std_msgs
)

find_package(Eigen REQUIRED)
find_package(cmake_modules REQUIRED)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
)

add_message_files(
  DIRECTORY
  msg
  FILES
  user_IDs.msg
  user_points.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect2_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions roscpp roslib tf tf_conversions
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(kinect2_tracker
   src/kinect2_tracker_node.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kinect2_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(NITE2_DIR ~/package_ws/NiTE-Linux-x64-2.2/)
set(NITE2_LIB ~/package_ws/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)
include_directories(./include/Openni2/)
include_directories(${NITE2_DIR}/Include)
include_directories(./include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

add_executable(kinect2_tracker_node src/kinect2_tracker_node.cpp)
target_link_libraries(kinect2_tracker_node OpenNI2 ${NITE2_LIB}  ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
