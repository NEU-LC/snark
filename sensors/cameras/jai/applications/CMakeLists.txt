source_group( jai-cat FILES jai-cat.cpp )
add_executable( jai-cat jai-cat.cpp )
target_link_libraries( jai-cat snark_jai snark_imaging boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
install( TARGETS jai-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
