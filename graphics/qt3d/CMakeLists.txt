SET( PROJECT "qt3d" )
SET( TARGET_NAME snark_graphics_${PROJECT} )
SET( dir ${SOURCE_CODE_BASE_DIR}/graphics/${PROJECT} )

if( Qt3D_VERSION EQUAL 1 )
    set( snark_qt3d_specific_dir "qt3d_v1" )
elseif( Qt3D_VERSION EQUAL 2 )
    set( snark_qt3d_specific_dir "qt3d_v2" )
else()
    message( FATAL_ERROR "Qt3D_VERSION is ${Qt3D_VERSION}, must be 1 or 2" )
endif()

file( GLOB source  ${dir}/*.cpp ${dir}/${snark_qt3d_specific_dir}/*.cpp )
file( GLOB include ${dir}/*.h   ${dir}/${snark_qt3d_specific_dir}/*.h )

if( Qt3D_VERSION EQUAL 1 )
    if( QT_VERSION_MAJOR EQUAL 4 )
        QT4_WRAP_CPP( snark_graphics_qt3d_moc ${dir}/${snark_qt3d_specific_dir}/view.h OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED )
    elseif( QT_VERSION_MAJOR EQUAL 5 )
        QT5_WRAP_CPP( snark_graphics_qt3d_moc ${dir}/${snark_qt3d_specific_dir}/view.h )
    endif()
endif()

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} ${snark_graphics_qt3d_moc} ${gl_source} )
ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${snark_graphics_qt3d_moc} ${gl_source} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )

if( QT_VERSION_MAJOR EQUAL 4 )
  target_link_libraries( ${TARGET_NAME} snark_math ${QT_LIBRARIES} ${Qt3D_LIB} ${snark_ALL_EXTERNAL_LIBRARIES} )
elseif( QT_VERSION_MAJOR EQUAL 5 )
  if( QT_VERSION_MINOR LESS 5 )
      target_link_libraries( ${TARGET_NAME}
                             snark_math
                             Qt5::3D
                             ${OPENGL_LIBRARY}
                             ${snark_ALL_EXTERNAL_LIBRARIES} )
  else()
      target_link_libraries( ${TARGET_NAME}
                             snark_math
                             Qt5::Widgets
                             Qt5::3DCore
                             ${OPENGL_LIBRARY}
                             ${snark_ALL_EXTERNAL_LIBRARIES} )
  endif()
endif()

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
