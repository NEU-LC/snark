SET( PROJECT "hokuyo_detail" )
SET( TARGET_NAME snark_${PROJECT} )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} snark_timing ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/hokuyo/detail )
INSTALL(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
