SET( PROJECT "geodesy" )
SET( TARGET_NAME snark_${PROJECT} )

FILE( GLOB source *.cpp)
FILE( GLOB includes *.h )
FILE( GLOB detail_source detail/GeographicConversions/*.cpp ) # quick and dirty
FILE( GLOB detail_includes detail/GeographicConversions/*.h ) # quick and dirty
SOURCE_GROUP( ${PROJECT} FILES ${source} ${includes} ${detail_source} ${detail_includes} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL( FILES ${detail_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/detail/GeographicConversions )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${detail_source} ${detail_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} snark_math_spherical_geometry ${snark_ALL_EXTERNAL_LIBRARIES} )
INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( agd84 )
ADD_SUBDIRECTORY( grs67 )
ADD_SUBDIRECTORY( wgs84 )
ADD_SUBDIRECTORY( applications )

if( snark_build_geodesy_solar )
    add_subdirectory( solar )
endif( snark_build_geodesy_solar )
