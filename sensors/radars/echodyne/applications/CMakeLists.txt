source_group( echodyne-cat FILES echodyne-cat.cpp )
add_executable( echodyne-cat echodyne-cat.cpp )
target_link_libraries( echodyne-cat snark_echodyne bnetplus )

source_group( echodyne-cmd FILES echodyne-cmd.cpp )
add_executable( echodyne-cmd echodyne-cmd.cpp )
target_link_libraries( echodyne-cmd snark_echodyne bnetplus )

install( TARGETS echodyne-cat echodyne-cmd
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
