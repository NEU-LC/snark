set( ros_path "/opt/ros/kinetic" CACHE STRING "path to ros kinetic" )
link_directories( "${ros_path}/lib" )
include_directories( "${ros_path}/include" )

set( ros_libs roscpp roscpp_serialization rostime )

source_group( points-to-ros FILES points-to-ros.cpp )
add_executable( points-to-ros points-to-ros.cpp )
target_link_libraries( points-to-ros ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES}  ${ros_libs} )

install( TARGETS points-to-ros
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

install( PROGRAMS ros-bag-to-bin ros-to-image DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
