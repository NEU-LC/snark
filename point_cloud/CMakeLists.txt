SET( PROJECT "point_cloud" )
SET( TARGET_NAME snark_${PROJECT} )
SET( DIR ${SOURCE_CODE_BASE_DIR}/${PROJECT} )

FILE( GLOB source ${DIR}/*.cpp)
FILE( GLOB includes ${DIR}/*.h)
FILE( GLOB impl_includes ${DIR}/impl/*.h )

SOURCE_GROUP( ${PROJECT} FILES ${source} ${includes} ${impl_includes} )
ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${impl_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
target_link_libraries( ${TARGET_NAME} snark_math )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT} )
INSTALL( FILES ${impl_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT}/impl )
INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

IF( BUILD_APPLICATIONS )
    ADD_SUBDIRECTORY( applications )
ENDIF( BUILD_APPLICATIONS )

IF( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
ENDIF( BUILD_TESTS )

IF( snark_build_point_cloud_pcl )
    ADD_SUBDIRECTORY( pcl )
ENDIF( snark_build_point_cloud_pcl )

