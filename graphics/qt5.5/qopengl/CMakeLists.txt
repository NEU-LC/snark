set( PROJECT "qopengl" )
set( TARGET_NAME snark_graphics_${PROJECT} )

if( QT_VERSION_MAJOR LESS 5 )
    message( FATAL_ERROR "snark graphics qopengl requires Qt 5.5 or higher (got vesion: ${QT_VERSION_MAJOR}.${QT_VERSION_MINOR})" )
elseif( QT_VERSION_MAJOR EQUAL 5 )
    if( QT_VERSION_MINOR LESS 5 )
        message( FATAL_ERROR "snark graphics qopengl requires Qt 5.5 or higher (got vesion: ${QT_VERSION_MAJOR}.${QT_VERSION_MINOR})" )
    endif()
endif()

if( NOT Qt3D_VERSION EQUAL 2 )
    message( FATAL_ERROR "Qt3D_VERSION is ${Qt3D_VERSION}, must be 2" )
endif()

file( GLOB source  *.cpp )
file( GLOB include *.h   )

# if( Qt3D_VERSION EQUAL 2 )
    QT5_WRAP_CPP( snark_graphics_qopengl_moc widget.h )
# endif()

source_group( ${TARGET_NAME} FILES ${source} ${includes} ${snark_graphics_qopengl_moc} )
add_library( ${TARGET_NAME} ${source} ${includes} ${snark_graphics_qopengl_moc} )
set_target_properties( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )

# Qt5::3DCore 
target_link_libraries( ${TARGET_NAME} snark_math Qt5::Widgets ${OPENGL_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} )

# install( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
install(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
