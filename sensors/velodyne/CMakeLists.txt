SET( PROJECT "velodyne" )
SET( TARGET_NAME snark_${PROJECT} )

INCLUDE( ${SOURCE_CODE_BASE_DIR}/CMakeFiles/FindPcap.cmake REQUIRED )
IF( ${PCAP_FOUND} )
    INCLUDE_DIRECTORIES( ${PCAP_INCLUDE_DIRS} )
    SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} ${PCAP_LIBRARIES} )
ENDIF( ${PCAP_FOUND} )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/*.cpp )
FILE( GLOB includes ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/*.h )
FILE( GLOB impl_source ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/impl/*.cpp )
FILE( GLOB impl_includes ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/impl/*.h )
FILE( GLOB thin_source ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/thin/*.cpp )
FILE( GLOB thin_includes ${SOURCE_CODE_BASE_DIR}/sensors/${PROJECT}/thin/*.h ) 

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes}
                                   ${impl_source} ${impl_includes}
                                   ${thin_source} ${thin_includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${impl_source} ${impl_includes} ${thin_source} ${thin_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/sensors/${PROJECT} )
INSTALL( FILES ${impl_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/sensors/${PROJECT}/impl )
INSTALL( FILES ${thin_includes} DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_INCLUDE_DIR}/sensors/${PROJECT}/thin )
INSTALL(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( applications )

if( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
endif( BUILD_TESTS )

