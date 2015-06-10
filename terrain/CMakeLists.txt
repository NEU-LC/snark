set( PROJECT "terrain" )
set( TARGET_NAME snark_${PROJECT} )
file( GLOB source *.cpp )
file( GLOB includes *.h )
#source_group( ${TARGET_NAME} FILES ${source} ${includes} )
add_library( ${TARGET_NAME} ${source}
                            ${includes}
                            $<TARGET_OBJECTS:snark_terrain_dem_srtm>
                            $<TARGET_OBJECTS:snark_terrain_dem> )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} snark_math_spherical_geometry ${snark_ALL_EXTERNAL_LIBRARIES} )

#install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

add_subdirectory( applications )
add_subdirectory( dem )
