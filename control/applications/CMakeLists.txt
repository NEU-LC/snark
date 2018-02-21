ADD_EXECUTABLE( control-command control-command.cpp )
TARGET_LINK_LIBRARIES( control-command ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} snark_control )
INSTALL( TARGETS control-command RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( control-error control-error.cpp )
TARGET_LINK_LIBRARIES( control-error ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} snark_control )
INSTALL( TARGETS control-error RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

INSTALL( PROGRAMS control-follow-waypoints DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

