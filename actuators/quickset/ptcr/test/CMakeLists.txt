SET( KIT quickset_ptcr )

FILE( GLOB source ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*test.cpp )
FILE( GLOB extras ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.cpp
    ${SOURCE_CODE_BASE_DIR}/actuators/quickset/ptcr/test/*.h )
LIST( REMOVE_ITEM extras ${source} )

ADD_EXECUTABLE( ${CMAKE_PROJECT_NAME}_test_${KIT} ${source} ${extras} )
TARGET_LINK_LIBRARIES(
    ${CMAKE_PROJECT_NAME}_test_${KIT}
    snark_quickset_ptcr
    ${snark_ALL_EXTERNAL_LIBRARIES}
    ${GTEST_BOTH_LIBRARIES}
    )
INSTALL(
    FILES ${PROJECT_BINARY_DIR}/bin/${CMAKE_PROJECT_NAME}_test_${KIT}
    PERMISSIONS WORLD_READ GROUP_READ OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
    DESTINATION ${snark_CPP_TESTS_INSTALL_DIR} )

add_test( NAME ${CMAKE_PROJECT_NAME}_test_${KIT} COMMAND ${CMAKE_PROJECT_NAME}_test_${KIT} WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
