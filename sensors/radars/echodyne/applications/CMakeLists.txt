source_group( echodyne-cat FILES echodyne-cat.cpp )
add_executable( echodyne-cat echodyne-cat.cpp )
target_link_libraries( echodyne-cat snark_echodyne bnetplus )

source_group( echodyne-command FILES echodyne-command.cpp )
add_executable( echodyne-command echodyne-command.cpp )
target_link_libraries( echodyne-command snark_echodyne bnetplus )

install( TARGETS echodyne-cat echodyne-command
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
