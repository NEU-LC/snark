source_group( ipx-cat FILES ipx-cat.cpp )
add_executable( ipx-cat ipx-cat.cpp )
#target_link_libraries( ipx-cat snark_ipx snark_imaging ${Boost_PROGRAM_OPTIONS_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )
target_link_libraries( ipx-cat
                       snark_ipx
                       snark_imaging
                       ${comma_ALL_LIBRARIES}
                       ${comma_ALL_EXTERNAL_LIBRARIES}
                       ${snark_ALL_EXTERNAL_LIBRARIES}
                       ${Boost_PROGRAM_OPTIONS_LIBRARY}
                       tbb )
install( TARGETS ipx-cat RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )