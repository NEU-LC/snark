SET(KIT timing)

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*_test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( timing_test ${source} ${extras} )

TARGET_LINK_LIBRARIES( timing_test snark_timing ${GTEST_BOTH_LIBRARIES} )
