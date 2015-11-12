SET(KIT batteries/ocean )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*_test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/${KIT}/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( ocean_test ${source} ${extras} )

TARGET_LINK_LIBRARIES( ocean_test snark_ocean ${Boost_SYSTEM_LIBRARY} ${comma_ALL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} )
