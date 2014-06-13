SET( dir ${SOURCE_CODE_BASE_DIR}/actuators/robot_arm/applications )
FILE( GLOB source   ${dir}/*.cpp)
FILE( GLOB includes ${dir}/*.h)
SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )

ADD_EXECUTABLE( robot-arm-daemon robot-arm-daemon.cpp )
TARGET_LINK_LIBRARIES( robot-arm-daemon snark_robot_arm snark_robot_arm_simulink snark_ocean boost_system ${comma_ALL_LIBRARIES} )
INSTALL( TARGETS robot-arm-daemon RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

