set( ros_path "/opt/ros/kinetic" CACHE STRING "path to ros, something like /opt/ros/kinetic" )
if( NOT EXISTS "${ros_path}")
    message( "${ros_path} not found; please specify ros path as something like /opt/ros/kinetic on your computer or set snark_build_ros to OFF" )
    set( ros_path "NOT FOUND" CACHE STRING "path to ros, something like /opt/ros/kinetic" )
endif( NOT EXISTS "${ros_path}")
link_directories( "${ros_path}/lib" )
include_directories( "${ros_path}/include" )

set( ros_libs roscpp roscpp_serialization rostime rosconsole )

# source_group( points-to-ros FILES points-to-ros.cpp )
add_executable( points-to-ros points-to-ros.cpp )
target_link_libraries( points-to-ros ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES}  ${ros_libs} )

# source_group( points-from-ros FILES points-from-ros.cpp )
add_executable( points-from-ros points-from-ros.cpp )
target_link_libraries( points-from-ros ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES}  ${ros_libs} )

install( TARGETS points-to-ros points-from-ros
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

install( PROGRAMS ros-bag-to-bin ros-to-image DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ros-to-csv DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ros-from-csv DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

