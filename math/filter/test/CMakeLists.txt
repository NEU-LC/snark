SET(KIT filter)

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/math/${KIT}/test/*test.cpp )

ADD_EXECUTABLE( test_${KIT} ${source} )

TARGET_LINK_LIBRARIES( test_${KIT} snark_math ${snark_ALL_EXTERNAL_LIBRARIES}  ${OpenCV_LIBS} ${GTEST_BOTH_LIBRARIES} )
