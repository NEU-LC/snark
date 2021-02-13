set( PROJECT "ntrip" )
set( TARGET_NAME snark_navigation_${PROJECT} )

file( GLOB source   ${dir}/*.cpp)
file( GLOB includes ${dir}/*.h)
source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_executable( ntrip-cat ntrip-cat.cpp )
target_link_libraries( ntrip-cat ${comma_ALL_LIBRARIES} )
set_target_properties( ntrip-cat PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS ntrip-cat RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

install( PROGRAMS ntrip-client ntrip-csv-to-gpgga DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR} )
