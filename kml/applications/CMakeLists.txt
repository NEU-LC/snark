set( dir ${SOURCE_CODE_BASE_DIR}/kml/applications )
file( GLOB source   ${dir}/*.cpp)
file( GLOB includes ${dir}/*.h)
source_group( ${TARGET_NAME} FILES ${source} ${includes} )

add_executable( csv-to-kml csv-to-kml.cpp )
target_link_libraries( csv-to-kml ${comma_ALL_LIBRARIES} )
install( TARGETS csv-to-kml RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
