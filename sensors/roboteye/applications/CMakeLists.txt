source_group( ocular-roboteye-to-csv FILES ocular-roboteye-thermal-cat.cpp ocular-roboteye-to-csv.cpp )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOCULAR_ROBOTEYE_GENICAM_DIR=\"${OCULAR_ROBOTEYE_GENICAM_DIR}\"" )

add_executable( ocular-roboteye-thermal-cat ocular-roboteye-thermal-cat.cpp )
target_link_libraries( ocular-roboteye-thermal-cat snark_roboteye )
install( TARGETS ocular-roboteye-thermal-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

configure_file( ocular-roboteye-thermal.template ocular-roboteye-thermal @ONLY )
install( PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/ocular-roboteye-thermal
         DESTINATION ${snark_INSTALL_BIN_DIR} )

add_executable( ocular-roboteye-to-csv ocular-roboteye-to-csv.cpp )
target_link_libraries( ocular-roboteye-to-csv snark_roboteye )
install( TARGETS ocular-roboteye-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
