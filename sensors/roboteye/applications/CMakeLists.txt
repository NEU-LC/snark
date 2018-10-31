source_group( ocular-roboteye-to-csv FILES ocular-roboteye-to-csv.cpp )
add_executable( ocular-roboteye-to-csv ocular-roboteye-to-csv.cpp )
target_link_libraries( ocular-roboteye-to-csv snark_roboteye )
install( TARGETS ocular-roboteye-to-csv
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
