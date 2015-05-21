ADD_EXECUTABLE( control-error control-error.cpp )
TARGET_LINK_LIBRARIES( control-error ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL( TARGETS control-error RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
