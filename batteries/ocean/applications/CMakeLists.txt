SET( dir ${SOURCE_CODE_BASE_DIR}/battery/ocean/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( ocean-to-csv ocean-to-csv.cpp )
TARGET_LINK_LIBRARIES( ocean-to-csv snark_ocean ${comma_ALL_LIBRARIES} )
set_target_properties( ocean-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
INSTALL( TARGETS ocean-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

