SET( PROJECT "velodyne" )
SET( TARGET_NAME snark_${PROJECT} )

INCLUDE( ${SOURCE_CODE_BASE_DIR}/CMakeFiles/FindPcap.cmake REQUIRED )
IF( ${PCAP_FOUND} )
    INCLUDE_DIRECTORIES( ${PCAP_INCLUDE_DIRS} )
    SET( snark_ALL_EXTERNAL_LIBRARIES ${snark_ALL_EXTERNAL_LIBRARIES} ${PCAP_LIBRARIES} )
ENDIF( ${PCAP_FOUND} )

FILE( GLOB source *.cpp )
FILE( GLOB includes *.h )
FILE( GLOB impl_source impl/*.cpp )
FILE( GLOB impl_includes impl/*.h )
FILE( GLOB hdl64_source hdl64/*.cpp )
FILE( GLOB hdl64_includes hdl64/*.h )
FILE( GLOB puck_source puck/*.cpp )
FILE( GLOB puck_includes puck/*.h )
FILE( GLOB thin_source thin/*.cpp )
FILE( GLOB thin_includes thin/*.h ) 

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes}
                                   ${hdl64_source} ${hdl64_includes}
                                   ${impl_source} ${impl_includes}
                                   ${puck_source} ${puck_includes}
                                   ${thin_source} ${thin_includes} )

ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes} ${hdl64_source} ${hdl64_includes} ${impl_source} ${impl_includes} ${puck_source} ${puck_includes} ${thin_source} ${thin_includes} )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )
TARGET_LINK_LIBRARIES( ${TARGET_NAME} snark_math ${snark_ALL_EXTERNAL_LIBRARIES} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT} )
INSTALL( FILES ${hdl64_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/hdl64 )
INSTALL( FILES ${impl_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/impl )
INSTALL( FILES ${puck_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/puck )
INSTALL( FILES ${thin_includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/sensors/lidars/${PROJECT}/thin )
INSTALL(
   TARGETS ${TARGET_NAME}
   RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
   LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
   ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)

ADD_SUBDIRECTORY( applications )

if( BUILD_TESTS )
    ADD_SUBDIRECTORY( test )
endif( BUILD_TESTS )

