set( PROJECT "ros" )
set( TARGET_NAME snark_${PROJECT} )
set( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

file( GLOB source ${dir}/*.cpp )
file( GLOB includes ${dir}/*.h )

add_library( ${TARGET_NAME} ${source} ${include} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
#target_link_libraries( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
