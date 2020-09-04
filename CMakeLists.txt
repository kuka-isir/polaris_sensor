cmake_minimum_required(VERSION 2.8.3)
project(polaris_sensor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  serial
  cmake_modules
)


#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS system filesystem REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polaris_sensor
  CATKIN_DEPENDS geometry_msgs roscpp serial sensor_msgs tf
)

include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/polaris_sensor.cpp
 )

add_executable(${PROJECT_NAME}_node src/polaris_ros.cpp)
add_executable(${PROJECT_NAME}_double_node src/polaris_ros_double.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}
 )
target_link_libraries(${PROJECT_NAME}_double_node
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}
 )


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_double_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)
