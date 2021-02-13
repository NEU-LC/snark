add_executable( robosense-to-csv robosense-to-csv.cpp )
target_link_libraries( robosense-to-csv snark_robosense snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( robosense-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS robosense-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
