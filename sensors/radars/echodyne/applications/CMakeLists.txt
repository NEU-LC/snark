source_group( echodyne-cat FILES echodyne-cat.cpp )
add_executable( echodyne-cat echodyne-cat.cpp )
set_target_properties( echodyne-cat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
target_link_libraries( echodyne-cat snark_echodyne bnetinterface bnetreader )

source_group( echodyne-command FILES echodyne-command.cpp )
add_executable( echodyne-command echodyne-command.cpp )
set_target_properties( echodyne-command PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
target_link_libraries( echodyne-command snark_echodyne bnetinterface bnetreader )

install( TARGETS echodyne-cat echodyne-command
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
