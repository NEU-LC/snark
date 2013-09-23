SET( KIT math )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*test.cpp )

ADD_EXECUTABLE( test_${KIT} ${source} )

TARGET_LINK_LIBRARIES( test_${KIT} snark_math snark_point_cloud ${GTEST_BOTH_LIBRARIES} pthread )
