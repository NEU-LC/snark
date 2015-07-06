add_executable( trimble-to-csv trimble-to-csv.cpp )
target_link_libraries( trimble-to-csv snark_navigation_trimble_bd9xx ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS trimble-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
