SET( KIT velodyne )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/sensors/${KIT}/test/*test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/sensors/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/sensors/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_${KIT}
                       snark_${KIT}
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
