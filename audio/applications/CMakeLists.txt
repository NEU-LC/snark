ADD_EXECUTABLE( audio-sample audio-sample.cpp )
TARGET_LINK_LIBRARIES( audio-sample ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( audio-sample PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS audio-sample RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
