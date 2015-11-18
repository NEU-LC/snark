SET( PROJECT "spherical_geometry" )

SET( TARGET_NAME snark_math_${PROJECT} )

FILE( GLOB includes *.h)

FILE( GLOB sources *.cpp )

source_group( ${TARGET_NAME} FILES ${sources} ${includes} )

# quick and dirty
#add_library( ${TARGET_NAME} OBJECT ${sources} ${includes} )    //OBJECT doesn't work on cmake 2.8.2
add_library( ${TARGET_NAME} ${sources} ${includes} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/math/${PROJECT} )
install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

IF( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
ENDIF( BUILD_TESTS )
