set( PROJECT "kml" )
set( TARGET_NAME snark_${PROJECT} )
set( DIR ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

FILE( GLOB source ${DIR}/*.cpp)
FILE( GLOB includes ${DIR}/*.h)

source_group( ${PROJECT} FILES ${source} ${includes} )
add_library( ${TARGET_NAME} ${source} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${comma_ALL_LIBRARIES} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL( FILES ${impl_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/impl )
INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
