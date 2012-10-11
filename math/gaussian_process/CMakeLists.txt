SET( PROJECT "gaussian_process" )
SET( TARGET_NAME snark_${PROJECT} )
SET( dir ${SOURCE_CODE_BASE_DIR}/math/${PROJECT} )

FILE( GLOB source ${dir}/*.cpp )
FILE( GLOB includes ${dir}/*.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

INSTALL( FILES ${includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/math/${PROJECT} )

IF( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
ENDIF( BUILD_TESTS )

