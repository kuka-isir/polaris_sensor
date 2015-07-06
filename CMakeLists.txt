cmake_minimum_required(VERSION 2.8.3)
project(polaris_sensor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  cmake_modules
)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polaris_sensor
  CATKIN_DEPENDS geometry_msgs roscpp serial sensor_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
   src/polaris_sensor.cpp
 )

 if(${OROCOS_TARGET} STREQUAL "xenomai" )
   find_package(Xenomai REQUIRED)
   set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI __XENO__)
 endif()

add_executable(${PROJECT_NAME} src/polaris_ros.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}_lib
 )


install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)
