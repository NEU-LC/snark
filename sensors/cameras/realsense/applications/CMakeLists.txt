source_group( realsense-cat FILES realsense-cat.cpp )
add_executable( realsense-cat realsense-cat.cpp )
target_link_libraries( realsense-cat snark_realsense snark_imaging ${snark_ALL_EXTERNAL_LIBRARIES} )
#target_link_libraries( realsense-cat snark_imaging realsense ${comma_ALL_LIBRARIES} )

install( TARGETS realsense-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
