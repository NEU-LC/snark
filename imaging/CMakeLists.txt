SET( PROJECT "imaging" )
SET( TARGET_NAME snark_${PROJECT} )
SET( DIR ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${PROJECT}/*.cpp )
FILE( GLOB includes ${SOURCE_CODE_BASE_DIR}/${PROJECT}/*.h )

FILE( GLOB cvmat_source ${SOURCE_CODE_BASE_DIR}/${PROJECT}/cv_mat/*.cpp )
FILE( GLOB cvmat_includes ${SOURCE_CODE_BASE_DIR}/${PROJECT}/cv_mat/*.h )

FILE( GLOB stereo_source ${SOURCE_CODE_BASE_DIR}/${PROJECT}/stereo/*.cpp )
FILE( GLOB stereo_includes ${SOURCE_CODE_BASE_DIR}/${PROJECT}/stereo/*.h )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${cvmat_source} ${cvmat_includes} ${stereo_source} ${stereo_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${comma_ALL_LIBRARIES} ${comma_ALL_EXTERNAL_LIBRARIES} ${OpenCV_LIBS} tbb )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL( FILES ${cvmat_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/cv_mat )
INSTALL( FILES ${stereo_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/stereo )
INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

IF( snark_BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
    ADD_SUBDIRECTORY( examples )
ENDIF( snark_BUILD_APPLICATIONS )

