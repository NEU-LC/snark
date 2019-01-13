if( BUILD_TESTS )
    add_subdirectory( test )
endif( BUILD_TESTS )

install( PROGRAMS geo-sun-position DESTINATION ${snark_INSTALL_BIN_DIR} )
