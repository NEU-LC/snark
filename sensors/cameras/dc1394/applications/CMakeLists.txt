ADD_EXECUTABLE( fire-cat fire-cat.cpp )
TARGET_LINK_LIBRARIES( fire-cat comma_name_value snark_dc1394 snark_imaging tbb ${Boost_PROGRAM_OPTIONS_LIBRARY} )

ADD_EXECUTABLE( point-grey point-grey.cpp )
TARGET_LINK_LIBRARIES( point-grey comma_name_value snark_dc1394 snark_imaging ${Boost_PROGRAM_OPTIONS_LIBRARY})

INSTALL( TARGETS fire-cat point-grey RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
