cmake_minimum_required(VERSION 3.0.2)
project(gnss_mapping)

find_package(PCL REQUIRED)

if(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
  SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
endif()


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  velodyne_pointcloud
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    velodyne_pointcloud
  DEPENDS PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

add_executable(gnss_mapping src/gnss_mapping.cpp)
target_link_libraries(gnss_mapping ${catkin_LIBRARIES})
add_dependencies(gnss_mapping ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    gnss_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)