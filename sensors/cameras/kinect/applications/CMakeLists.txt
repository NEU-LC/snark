source_group( kinect-cat FILES kinect-cat.cpp )
add_executable( kinect-cat kinect-cat.cpp )
# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
include_directories("/usr/include/libusb-1.0/")
include_directories(${freenect2_INCLUDE_DIRS})
# Include directories to get freenect headers
include_directories($ENV{HOME}/freenect2/include)

target_link_libraries( kinect-cat snark_imaging ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} ${freenect2_LIBRARIES})
install( TARGETS kinect-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
