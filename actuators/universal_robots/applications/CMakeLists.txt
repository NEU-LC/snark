SET( dir ${SOURCE_CODE_BASE_DIR}/actuators/universal_robots/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( ur-arm-control ur-arm-control.cpp )
TARGET_LINK_LIBRARIES( ur-arm-control snark_robotic_arm snark_robotic_arm_simulink boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS ur-arm-control RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

ADD_EXECUTABLE( ur-arm-status ur-arm-status.cpp )
TARGET_LINK_LIBRARIES( ur-arm-status snark_robotic_arm snark_ur5_transforms snark_ur5_transforms_simulink snark_robotic_arm_simulink boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS ur-arm-status RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

