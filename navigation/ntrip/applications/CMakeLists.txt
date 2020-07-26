FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

add_executable( ntrip-cat ntrip-cat.cpp )
target_link_libraries( ntrip-cat ${comma_ALL_LIBRARIES} )
install( TARGETS ntrip-cat
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

INSTALL( PROGRAMS
             ntrip-client
             ntrip-csv-to-gpgga
             ntrip-status
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} )
