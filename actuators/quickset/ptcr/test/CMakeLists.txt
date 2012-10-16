FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.cpp
                  ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_quickset_ptcr ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_quickset_ptcr
                       snark_quickset_ptcr
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${GTEST_BOTH_LIBRARIES}
                     )
