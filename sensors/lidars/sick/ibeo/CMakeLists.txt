SET( PROJECT "sick" )
SET( TARGET_NAME snark_${PROJECT}_ibeo )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_LIBRARY( ${TARGET_NAME} OBJECT ${source} ${includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
set_target_properties( ${TARGET_NAME} PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/ibeo )
