add_executable( advanced-navigation-cat advanced-navigation-cat.cpp )
target_link_libraries( advanced-navigation-cat snark_navigation_advanced_navigation tbb ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( advanced-navigation-cat PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS advanced-navigation-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( advanced-navigation-orientus-to-csv advanced-navigation-orientus-to-csv.cpp )
target_link_libraries( advanced-navigation-orientus-to-csv snark_navigation_advanced_navigation ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( advanced-navigation-orientus-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS advanced-navigation-orientus-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( advanced-navigation-spatial-to-csv advanced-navigation-spatial-to-csv.cpp )
target_link_libraries( advanced-navigation-spatial-to-csv snark_navigation_advanced_navigation tbb ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( advanced-navigation-spatial-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS advanced-navigation-spatial-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )

add_executable( advanced-navigation-to-csv advanced-navigation-to-csv.cpp )
target_link_libraries( advanced-navigation-to-csv snark_navigation_advanced_navigation tbb ${comma_ALL_LIBRARIES} ${snark_ALL_EXTERNAL_LIBRARIES} )
set_target_properties( advanced-navigation-to-csv PROPERTIES LINK_FLAGS_RELEASE -s )
install( TARGETS advanced-navigation-to-csv RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
