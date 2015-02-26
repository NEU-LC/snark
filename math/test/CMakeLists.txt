SET( KIT math )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*test.cpp )

ADD_EXECUTABLE( test_${KIT} ${source} )

TARGET_LINK_LIBRARIES( test_${KIT} snark_math ${snark_ALL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} pthread )
