SET( KIT "geometry" )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/math/${KIT}/test/test_*.cpp )

ADD_EXECUTABLE( test_${KIT} ${source} )

TARGET_LINK_LIBRARIES( test_${KIT} ${comma_ALL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} snark_math boost_system boost_thread  pthread ${comma_ALL_LIBRARIES})

