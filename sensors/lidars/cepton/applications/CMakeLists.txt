
# IF( NOT WIN32 )
# #comma::io::select doesn't compile in windows
source_group( cepton-cat FILES cepton-cat.cpp )
add_executable( cepton-cat cepton-cat.cpp )
target_link_libraries( cepton-cat ${snark_ALL_EXTERNAL_LIBRARIES} cepton_sdk snark_cepton )
install( TARGETS cepton-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )

# ENDIF(NOT WIN32 )

