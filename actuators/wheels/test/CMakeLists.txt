ADD_EXECUTABLE( test_wheels test_wheels.cpp )
TARGET_LINK_LIBRARIES( test_wheels snark_wheels ${snark_ALL_EXTERNAL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
