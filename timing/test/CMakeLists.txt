SET(KIT timing)

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*_test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${KIT} ${source} ${extras} )

TARGET_LINK_LIBRARIES( test_${KIT} snark_timing ${GTEST_BOTH_LIBRARIES} )
INSTALL ( 
		FILES ${PROJECT_BINARY_DIR}/bin/test_${KIT}
		PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE WORLD_EXECUTE
		DESTINATION ${snark_TESTS_DIR} )