add_executable( control-from-console control-from-console.cpp )
target_link_libraries( control-from-console ${snark_ALL_EXTERNAL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS control-from-console RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
