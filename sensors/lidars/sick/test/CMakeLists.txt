SET( KIT sick )

FILE( GLOB source *test.cpp )
FILE( GLOB extras *.cpp
                  *.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test${KIT}
                       snark_${KIT}
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
