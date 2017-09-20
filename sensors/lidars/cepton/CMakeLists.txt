set( PROJECT "cepton" )
set( TARGET_NAME snark_${PROJECT} )
file( GLOB includes *.h)
file( GLOB sources *.cpp )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )
add_library( ${TARGET_NAME} ${sources} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} cepton_sdk )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory(applications)
