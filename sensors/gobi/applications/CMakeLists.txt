SOURCE_GROUP( gobi-cat FILES gobi-cat.cpp )
ADD_EXECUTABLE( gobi-cat gobi-cat.cpp )
TARGET_LINK_LIBRARIES( gobi-cat snark_gobi snark_imaging xeneth boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

SOURCE_GROUP( gobi-to-temperature FILES gobi-to-temperature.cpp )
ADD_EXECUTABLE( gobi-to-temperature gobi-to-temperature.cpp )
TARGET_LINK_LIBRARIES( gobi-to-temperature snark_gobi snark_imaging xeneth boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

INSTALL( TARGETS gobi-cat gobi-to-temperature
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
