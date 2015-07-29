SET( KIT velodyne )

FILE( GLOB source *test.cpp )
FILE( GLOB extras *.cpp
                  *.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_${KIT}
                       snark_velodyne
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
