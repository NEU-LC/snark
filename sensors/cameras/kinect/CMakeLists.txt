set( PROJECT "kinect" )
set( TARGET_NAME snark_${PROJECT} )

# file( GLOB source *.cpp )
# file( GLOB includes *.h )
# 
# source_group( ${TARGET_NAME} FILES ${source} ${includes} )
# 
# ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
# SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
# TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} xeneth )
# 
# INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/cameras/${PROJECT} )
# INSTALL(
#    TARGETS ${TARGET_NAME}
#    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
#    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
#    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
# )

add_subdirectory( applications )

