add_executable( test_nmea test_nmea.cpp )
target_link_libraries( test_nmea snark_navigation_nmea ${snark_ALL_EXTERNAL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
