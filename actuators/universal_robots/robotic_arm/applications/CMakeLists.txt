SET( dir ${SOURCE_CODE_BASE_DIR}/actuators/ur10/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( snark-ur10-control snark-ur10-control.cpp )
TARGET_LINK_LIBRARIES( snark-ur10-control snark_robotic_arm snark_robotic_arm_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur10-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( snark-ur10-status snark-ur10-status.cpp )
TARGET_LINK_LIBRARIES( snark-ur10-status snark_robotic_arm snark_robotic_arm_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur10-status RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( snark-ur10-from-console snark-ur10-from-console.cpp )
TARGET_LINK_LIBRARIES( snark-ur10-from-console snark_robotic_arm snark_robotic_arm_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur10-from-console RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

