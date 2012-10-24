SET( PROJECT "math" )
SET( TARGET_NAME snark_${PROJECT} )
SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

FILE( GLOB filter_source ${dir}/filter/*.cpp )
FILE( GLOB filter_includes ${dir}/filter/*.h )
FILE( GLOB fft_source ${dir}/fft/*.cpp )
FILE( GLOB fft_includes ${dir}/fft/*.h )
FILE( GLOB gaussian_process_source ${dir}/gaussian_process/*.cpp )
FILE( GLOB gaussian_process_includes ${dir}/gaussian_process/*.h )
FILE( GLOB math_source ${dir}/*.cpp )
FILE( GLOB math_includes ${dir}/*.h )

SET( source ${math_source} ${filter_source} ${gaussian_process_source} ${fft_source} )
SET( includes ${math_includes} ${filter_includes} ${gaussian_process_includes} ${fft_includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${filter_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/filter )
INSTALL( FILES ${fft_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/fft )
INSTALL( FILES ${gaussian_process_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/gaussian_process )
INSTALL( FILES ${math_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )

INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( filter )
ADD_SUBDIRECTORY( fft )
ADD_SUBDIRECTORY( gaussian_process )

IF( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
ENDIF( BUILD_TESTS )

ADD_SUBDIRECTORY( applications )

