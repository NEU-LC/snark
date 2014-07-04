SET( dir ${SOURCE_CODE_BASE_DIR}/actuators/ur10/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( snark-ur10-control snark-ur10-control.cpp )
TARGET_LINK_LIBRARIES( snark-ur10-control snark_ur10 snark_ur10_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur10-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( snark-ur10-status snark-ur10-status.cpp )
TARGET_LINK_LIBRARIES( snark-ur10-status snark_ur10 snark_ur10_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur10-status RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

