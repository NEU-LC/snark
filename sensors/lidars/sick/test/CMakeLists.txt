SET( KIT sick )

FILE( GLOB source *test.cpp )
FILE( GLOB extras *.cpp
                  *.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_${KIT}
                       snark_${KIT}
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
INSTALL ( 
		FILES ${PROJECT_BINARY_DIR}/bin/test_${KIT}
		PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
		DESTINATION ${snark_TESTS_DIR} )