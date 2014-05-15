SET( dir ${SOURCE_CODE_BASE_DIR}/battery/ocean/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( ocean-to-csv ocean-to-csv.cpp )
TARGET_LINK_LIBRARIES( ocean-to-csv snark_battery ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS ocean-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

