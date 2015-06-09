ADD_EXECUTABLE( test_control test_control.cpp )
TARGET_LINK_LIBRARIES( test_control ${GTEST_BOTH_LIBRARIES} ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
