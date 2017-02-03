source_group( kinect-cat FILES kinect-cat.cpp )
add_executable( kinect-cat kinect-cat.cpp )
target_link_libraries( kinect-cat snark_imaging ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS kinect-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
