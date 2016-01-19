SET(KIT batteries/ocean )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*_test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_ocean ${source} ${extras} )

TARGET_LINK_LIBRARIES( test_ocean snark_ocean ${Boost_SYSTEM_LIBRARY} ${comma_ALL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
INSTALL ( 
		FILES ${PROJECT_BINARY_DIR}/bin/test_ocean
		PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
		DESTINATION ${snark_TESTS_DIR} )