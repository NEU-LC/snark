SOURCE_GROUP( gobi-cat FILES gobi-cat.cpp )
ADD_EXECUTABLE( gobi-cat gobi-cat.cpp )
TARGET_LINK_LIBRARIES( gobi-cat snark_gobi snark_imaging xeneth boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

SOURCE_GROUP( gobi-callback FILES gobi-callback.cpp )
ADD_EXECUTABLE( gobi-callback gobi-callback.cpp )
TARGET_LINK_LIBRARIES( gobi-callback snark_gobi snark_imaging xeneth boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

INSTALL( TARGETS gobi-cat gobi-callback
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
