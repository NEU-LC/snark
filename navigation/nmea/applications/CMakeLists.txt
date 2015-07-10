add_executable( nmea-to-csv nmea-to-csv.cpp )
target_link_libraries( nmea-to-csv snark_navigation_nmea ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS nmea-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
