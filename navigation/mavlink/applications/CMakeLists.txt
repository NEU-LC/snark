add_executable( mavlink-to-csv mavlink-to-csv.cpp )
target_link_libraries( mavlink-to-csv ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS mavlink-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
