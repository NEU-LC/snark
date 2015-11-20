SET( KIT gaussian_process )
set( DIR ${SOURCE_CODE_BASE_DIR}/math/${KIT}/test )
FILE( GLOB source ${DIR}/*test.cpp )
FILE( GLOB extras ${DIR}/*.cpp ${DIR}/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_${KIT} ${comma_ALL_LIBRARIES} snark_math ${GTEST_BOTH_LIBRARIES} )
INSTALL ( 
		FILES ${PROJECT_BINARY_DIR}/bin/test_${KIT}
		PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE WORLD_EXECUTE
		DESTINATION ${snark_TESTS_DIR} )