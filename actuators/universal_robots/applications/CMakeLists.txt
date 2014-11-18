SET( dir ${SOURCE_CODE_BASE_DIR}/actuators/universal_robots/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( snark-ur5-control snark-ur5-control.cpp )
TARGET_LINK_LIBRARIES( snark-ur5-control snark_robotic_arm snark_robotic_arm_simulink boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur5-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( snark-ur5-status snark-ur5-status.cpp )
TARGET_LINK_LIBRARIES( snark-ur5-status snark_robotic_arm snark_ur5_transforms snark_ur5_transforms_simulink snark_robotic_arm_simulink boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur5-status RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( snark-ur5-from-console snark-ur5-from-console.cpp )
TARGET_LINK_LIBRARIES( snark-ur5-from-console snark_robotic_arm snark_robotic_arm_simulink boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS snark-ur5-from-console RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

