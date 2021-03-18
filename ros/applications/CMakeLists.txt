set( dir ${SOURCE_CODE_BASE_DIR}/ros/applications )

set( ros_path "/opt/ros/melodic" CACHE STRING "path to ros, something like /opt/ros/melodic" )
if( NOT EXISTS "${ros_path}")
    message( "${ros_path} not found; please specify ros path as something like /opt/ros/melodic on your computer or set snark_build_ros to OFF" )
    set( ros_path "NOT FOUND" CACHE STRING "path to ros, something like /opt/ros/melodic" FORCE )
endif( NOT EXISTS "${ros_path}")

link_directories( "${ros_path}/lib" )
include_directories( "${ros_path}/include" )

set( ros_libs roscpp roscpp_serialization rosbag rosbag_storage rostime rosconsole console_bridge cpp_common )

file( GLOB detail_source ${dir}/detail/*.cpp )

add_executable( ros-array ros-array.cpp ${detail_source} )
target_link_libraries( ros-array ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES} )
set_target_properties( ros-array PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( ros-image ros-image.cpp ${detail_source} )
target_link_libraries( ros-image ${ros_libs} snark_imaging ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES} )
set_target_properties( ros-image PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( points-from-ros points-from-ros.cpp ${detail_source} )
target_link_libraries( points-from-ros ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES} )
set_target_properties( points-from-ros PROPERTIES LINK_FLAGS_RELEASE -s )

add_executable( points-to-ros points-to-ros.cpp )
target_link_libraries( points-to-ros ${ros_libs} ${snark_ALL_EXTERNAL_LIBRARIES} ${catkin_LIBRARIES} )
set_target_properties( points-to-ros PROPERTIES LINK_FLAGS_RELEASE -s )

install( TARGETS ros-array ros-image points-from-ros points-to-ros
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ros-from-csv ros-to-csv
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
