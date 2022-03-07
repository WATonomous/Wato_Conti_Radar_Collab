cmake_minimum_required(VERSION 2.8.3)
project(radar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  pcl_conversions
  pcl_ros
  tf
  visualization_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   SensorCfg.msg
   EthCfg.msg
   RadarPacket.msg
   SensorStatus.msg
   RadarDetection.msg
   BoundingBoxList.msg
   BoundingBox.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES radar_driver
  CATKIN_DEPENDS roscpp rospy std_msgs tf
)

## Vars to build with
set(extra_LIBRARIES -lpcap -lpthread) #Packet Cap, Capabilities, Mutexes
set(CMAKE_CXX_FLAGS "-std=c++11 -w -g -O3 ${CMAKE_CXX_FLAGS}") #Ignore all warnings, debug flags, optimize
set(RADAR_UNFILTER_SOURCE src/UnfilteredMain.cpp    src/parser.cpp          src/sniffer.cpp)
set(RADAR_FILTER_SOURCE   src/radarPublisher.cpp    src/processPacket.cpp   src/FilteredMain.cpp)
set(VIS_SOURCE            src/PointCloudConverter.cpp)
set(MULTI_RADAR src/radarDetection.cpp)
set(DBSCAN_SOURCE src/radar_clustering.cpp src/dbscan.cpp src/cluster.cpp src/point.cpp)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include  ${catkin_INCLUDE_DIRS} )

## Unfiltered Publisher
add_executable(radar_publisher ${RADAR_UNFILTER_SOURCE}  )

target_link_libraries(radar_publisher
                      ${catkin_LIBRARIES}
                      ${extra_LIBRARIES}
                      )

add_dependencies(radar_publisher radar_driver_generate_messages_cpp ${radar_driver_EXPORTED_TARGETS})

## Filtered Publisher
add_executable(radar_processor ${RADAR_FILTER_SOURCE}  )

target_link_libraries(radar_processor
                      ${catkin_LIBRARIES}
                      ${extra_LIBRARIES}
                      )

add_dependencies(radar_processor radar_driver_generate_messages_cpp ${radar_driver_EXPORTED_TARGETS})

## Point Cloud Generator
add_executable(radar_visualizer ${VIS_SOURCE} )

target_link_libraries(radar_visualizer ${catkin_LIBRARIES} )

add_dependencies(radar_visualizer radar_driver_generate_messages_cpp ${radar_driver_EXPORTED_TARGETS})

## Multi Radar Infrastructure
add_executable(multi_radar ${MULTI_RADAR}  )

target_link_libraries(multi_radar
                      ${catkin_LIBRARIES}
                      ${extra_LIBRARIES}
                      )

add_dependencies(multi_radar radar_driver_generate_messages_cpp ${radar_driver_EXPORTED_TARGETS})

#RADAR POINTS TEST 
add_executable(radar_points src/radar_point_detections.cpp)
target_link_libraries(radar_points ${catkin_LIBRARIES})
add_dependencies(radar_points radar_driver_generate_messages_cpp)

#BOUNDING BOX RVIZ TEST
add_executable(radar_clustering ${DBSCAN_SOURCE})
target_link_libraries(radar_clustering ${catkin_LIBRARIES})
add_dependencies(radar_clustering radar_driver_generate_messages_cpp)

