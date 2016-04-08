source_group( vimba-cat FILES vimba-cat.cpp )
add_executable( vimba-cat vimba-cat.cpp )
target_link_libraries( vimba-cat snark_vimba snark_imaging VimbaCPP ${comma_ALL_LIBRARIES} )

install( TARGETS vimba-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
