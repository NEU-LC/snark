SET( PROJECT "math" )
SET( TARGET_NAME snark_${PROJECT} )
SET( dir ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

IF( CMAKE_SIZEOF_VOID_P EQUAL 4 )
    # disable eigen vectorization on 32 bit machines
    # see http://eigen.tuxfamily.org/index.php?title=FAQ#Vectorization
    ADD_DEFINITIONS( -DEIGEN_DONT_VECTORIZE )
    ADD_DEFINITIONS( -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT )
    ADD_DEFINITIONS( -DEIGEN_DONT_ALIGN_STATICALLY )
ENDIF()

FILE( GLOB filter_source ${dir}/filter/*.cpp )
FILE( GLOB filter_includes ${dir}/filter/*.h )
FILE( GLOB fft_source ${dir}/fft/*.cpp )
FILE( GLOB fft_includes ${dir}/fft/*.h )
FILE( GLOB gaussian_process_source ${dir}/gaussian_process/*.cpp )
FILE( GLOB gaussian_process_includes ${dir}/gaussian_process/*.h )
FILE( GLOB spherical_polygon_source ${dir}/spherical_polygon/*.cpp )
FILE( GLOB spherical_polygon_includes ${dir}/spherical_polygon/*.h )
FILE( GLOB math_source ${dir}/*.cpp )
FILE( GLOB math_includes ${dir}/*.h )

SET( source ${math_source} ${filter_source} ${gaussian_process_source} ${fft_source} ${spherical_polygon_source} )
SET( includes ${math_includes} ${filter_includes} ${gaussian_process_includes} ${fft_includes} ${spherical_polygon_include} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${filter_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/filter )
INSTALL( FILES ${fft_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/fft )
INSTALL( FILES ${gaussian_process_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/gaussian_process )
INSTALL( FILES ${spherical_polygon_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/spherical_polygon )
INSTALL( FILES ${math_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )

INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( filter )
ADD_SUBDIRECTORY( fft )
ADD_SUBDIRECTORY( gaussian_process )
ADD_SUBDIRECTORY( spherical_polygon )

IF( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
ENDIF( BUILD_TESTS )

ADD_SUBDIRECTORY( applications )

