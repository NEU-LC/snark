SET(KIT actuators/robot_arm )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/test_*.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_robot_arm ${source} ${extras} )

TARGET_LINK_LIBRARIES( test_robot_arm snark_robot_arm boost_system ${comma_ALL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
