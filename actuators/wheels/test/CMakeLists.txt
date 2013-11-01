FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/actuators/wheels/test/*test.cpp )

ADD_EXECUTABLE( test_wheel_command ${source})
TARGET_LINK_LIBRARIES( test_wheel_command
                       snark_wheels
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
