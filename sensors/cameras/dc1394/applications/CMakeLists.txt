add_executable( fire-cat fire-cat.cpp )
target_link_libraries( fire-cat comma_name_value snark_dc1394 snark_imaging tbb ${Boost_PROGRAM_OPTIONS_LIBRARY} )

add_executable( point-grey point-grey.cpp )
target_link_libraries( point-grey comma_name_value snark_dc1394 snark_imaging ${Boost_PROGRAM_OPTIONS_LIBRARY} )

install( TARGETS fire-cat point-grey RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
