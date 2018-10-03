SET( dir ${SOURCE_CODE_BASE_DIR}/geodesy/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( geo-calc geo-calc.cpp )
TARGET_LINK_LIBRARIES( geo-calc snark_geodesy )
INSTALL( TARGETS geo-calc RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
