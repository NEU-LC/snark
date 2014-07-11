ADD_EXECUTABLE( test_wheel_command test_wheel_command.cpp )
TARGET_LINK_LIBRARIES( test_wheel_command snark_wheels ${snark_ALL_EXTERNAL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )

ADD_EXECUTABLE( test_wheels test_wheels.cpp )
TARGET_LINK_LIBRARIES( test_wheels snark_wheels ${snark_ALL_EXTERNAL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
