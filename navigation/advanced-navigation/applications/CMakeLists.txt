add_executable( advanced-navigation-spatial-to-csv advanced-navigation-spatial-to-csv.cpp )
target_link_libraries( advanced-navigation-spatial-to-csv snark_navigation_advanced_navigation tbb ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
install( TARGETS advanced-navigation-spatial-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

