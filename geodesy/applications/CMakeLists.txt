SET( dir ${SOURCE_CODE_BASE_DIR}/geodesy/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

add_executable( geo-calc geo-calc.cpp )
target_link_libraries( geo-calc ${snark_ALL_LIBRARIES} ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS geo-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )



