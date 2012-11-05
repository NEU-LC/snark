ADD_EXECUTABLE( fire-cat fire-cat.cpp )
TARGET_LINK_LIBRARIES( fire-cat snark_dc1394 snark_imaging tbb boost_program_options )

INSTALL( TARGETS fire-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
