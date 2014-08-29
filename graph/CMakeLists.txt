SET( PROJECT "graph" )
SET( TARGET_NAME snark_${PROJECT} )
SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )

add_custom_target( ${TARGET_NAME} ${source} ${includes} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( snark_BUILD_APPLICATIONS )
