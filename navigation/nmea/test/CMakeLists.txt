add_executable( test_nmea test_nmea.cpp )
target_link_libraries( test_nmea snark_navigation_nmea ${snark_ALL_EXTERNAL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
INSTALL ( 
		FILES ${PROJECT_BINARY_DIR}/bin/test_nmea
		PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
		DESTINATION ${snark_TESTS_DIR} )