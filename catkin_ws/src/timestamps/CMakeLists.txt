cmake_minimum_required (VERSION 2.8.3)
project (timestamps)

############
## Catkin ##
############

set(DEPS
  message_filters
  geometry_msgs
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp rospy rosbag std_msgs
   cmake_modules
  ${DEPS}
)
find_package(Eigen REQUIRED)
#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES
##    Odome.msg
##    GPSStatus.msg
##    GPSFix.msg
    
#)

generate_messages(DEPENDENCIES 
  geometry_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    ${DEPS}
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}/timestamps src/timestamps.cpp)
set_target_properties(${PROJECT_NAME}/timestamps PROPERTIES OUTPUT_NAME "timestamps")
target_link_libraries(${PROJECT_NAME}/timestamps ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}/timestamps
  ${PROJECT_NAME}_generate_messages_cpp 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${PROJECT_NAME}/timestamps
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/fix_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
