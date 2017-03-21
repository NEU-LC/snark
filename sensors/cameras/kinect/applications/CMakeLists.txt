source_group( kinect-cat FILES kinect-cat.cpp )

set( BINARY_DIR "${CMAKE_BINARY_DIR}/sensors/cameras/kinect/applications" )
add_custom_target( kinect.conf ) # quick and dirty
add_custom_command( TARGET kinect.conf
                    POST_BUILD
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/kinect-generate-ld-conf ARGS "~/freenect2/lib" )
install( FILES ${BINARY_DIR}/kinect.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/kinect: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )


add_executable( kinect-cat kinect-cat.cpp )
add_dependencies( kinect-cat kinect.conf )

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
include_directories("/usr/include/libusb-1.0/")
include_directories(${freenect2_INCLUDE_DIRS})
# Include directories to get freenect headers
include_directories($ENV{HOME}/freenect2/include)

target_link_libraries( kinect-cat snark_imaging ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${freenect2_LIBRARIES})
install( TARGETS kinect-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )





