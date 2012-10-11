SET( kit gaussian_process )
set( dir ${SOURCE_CODE_BASE_DIR}/math/${kit}/test )
FILE( GLOB source ${dir}/*test.cpp )
FILE( GLOB extras ${dir}/*.cpp ${dir}/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( test_${kit} ${source} ${extras} )
TARGET_LINK_LIBRARIES( test_${kit} ${comma_ALL_LIBRARIES} snark_math ${GTEST_BOTH_LIBRARIES} )
