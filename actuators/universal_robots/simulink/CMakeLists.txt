SET( PROJECT "robotic_arm_simulink" )
SET( PROJECT_PATH "actuators/universal_robots/simulink" )
SET( TARGET_NAME snark_${PROJECT} )

FILE( GLOB includes ${SOURCE_CODE_BASE_DIR}/${PROJECT_PATH}/*.h )
FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/${PROJECT_PATH}/*.c )

SOURCE_GROUP( ${TARGET_NAME} FILES ${source} ${includes} )
ADD_LIBRARY( ${TARGET_NAME} ${source} ${includes}  )
SET_TARGET_PROPERTIES( ${TARGET_NAME} PROPERTIES ${snark_LIBRARY_PROPERTIES} )

INSTALL( FILES ${includes} DESTINATION ${snark_INSTALL_INCLUDE_DIR}/${PROJECT_PATH} )
INSTALL(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime     # .exe, .dll
    LIBRARY DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Runtime     # .so, mod.dll
    ARCHIVE DESTINATION ${snark_INSTALL_LIB_DIR} COMPONENT Development # .a, .lib
)
