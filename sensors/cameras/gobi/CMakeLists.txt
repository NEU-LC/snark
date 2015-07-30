SET( PROJECT "gobi" )
SET( TARGET_NAME snark_${PROJECT} )

INCLUDE( ${SOURCE_CODE_BASE_DIR}/CMakeFiles/FindXenicsGobi640GigE.cmake REQUIRED )
INCLUDE_DIRECTORIES( ${XenicsGobi640GigE_INCLUDE_DIR} )
LINK_DIRECTORIES( ${XenicsGobi640GigE_LINK_DIRECTORIES} )

ADD_DEFINITIONS( -D_x64 )
if( UNIX )
    add_definitions( -D_LINUX )
elseif( __WIN32__ )
    add_definitions( -MSC_VER )
endif( UNIX )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} xeneth )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/cameras/${PROJECT} )
INSTALL(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( applications )
