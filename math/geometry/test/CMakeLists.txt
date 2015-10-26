SET( KIT geometry )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/math/${KIT}/test/*test.cpp )

ADD_EXECUTABLE( test_${KIT} ${source} )

TARGET_LINK_LIBRARIES( test_${KIT} snark_math_geometry ${GTEST_BOTH_LIBRARIES} pthread )
