SET( PROJECT "sick" )
SET( TARGET_NAME snark_${PROJECT}_ibeo )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_LIBRARY( ${TARGET_NAME} OBJECT ${source} ${includes} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/ibeo )
