add_executable( dem-to-csv dem-to-csv.cpp )
target_link_libraries( dem-to-csv snark_terrain ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS dem-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
