set( PROJECT "wgs84" )

FILE( GLOB includes *.h)

FILE( GLOB sources *.cpp )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/geodesy/${PROJECT} )
