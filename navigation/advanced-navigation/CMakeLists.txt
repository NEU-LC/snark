set( PROJECT "advanced_navigation" )
set( TARGET_NAME snark_navigation_${PROJECT} )
file( GLOB includes *.h)
file( GLOB sources *.cpp )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )
add_library( ${TARGET_NAME} ${sources} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

# install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/navigation/${PROJECT} )

add_subdirectory( applications )

# if( snark_BUILD_TESTS )
#     add_subdirectory( test )
# endif( snark_BUILD_TESTS )

