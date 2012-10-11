SET( PROJECT "fft" )
SET( dir ${SOURCE_CODE_BASE_DIR}/math/${PROJECT} )

FILE( GLOB source ${dir}/*.cpp )
FILE( GLOB includes ${dir}/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

INSTALL( FILES ${includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/Math/${PROJECT} )


