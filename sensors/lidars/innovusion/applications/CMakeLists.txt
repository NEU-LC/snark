source_group( innovusion-cat FILES innovusion-cat.cpp )
add_executable( innovusion-cat innovusion-cat.cpp )
set_target_properties( innovusion-cat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
target_link_libraries( innovusion-cat snark_innovusion ${Innovusion_LIBRARIES} )

install( TARGETS innovusion-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
