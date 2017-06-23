source_group( kinect-cat FILES kinect-cat.cpp )

set( BINARY_DIR "${CMAKE_BINARY_DIR}/sensors/cameras/kinect/applications" )

# quick and dirty
get_filename_component( freenect2_LIB_DIR ${freenect2_DIR} DIRECTORY )
get_filename_component( freenect2_LIB_DIR ${freenect2_LIB_DIR} DIRECTORY )

add_custom_target( kinect.conf ) # quick and dirty
add_custom_command( TARGET kinect.conf
                    POST_BUILD
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/kinect-generate-ld-conf ARGS ${freenect2_LIB_DIR} )
install( FILES ${BINARY_DIR}/kinect.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/kinect: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )


add_executable( kinect-cat kinect-cat.cpp )
add_dependencies( kinect-cat kinect.conf )

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${freenect2_DIR})

# todo: use variable from the main cmake instead of hardcoded path
include_directories("/usr/include/libusb-1.0/")

include_directories(${freenect2_INCLUDE_DIRS})

target_link_libraries( kinect-cat snark_imaging ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCL_LIBRARIES} )
install( TARGETS kinect-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
