SOURCE_GROUP( gige-cat FILES gige-cat.cpp )
ADD_EXECUTABLE( gige-cat gige-cat.cpp )
TARGET_LINK_LIBRARIES( gige-cat snark_gige snark_imaging PvAPI boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

SOURCE_GROUP( gige-callback FILES gige-callback.cpp )
ADD_EXECUTABLE( gige-callback gige-callback.cpp )
TARGET_LINK_LIBRARIES( gige-callback snark_gige snark_imaging PvAPI boost_program_options ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

INSTALL( TARGETS gige-cat gige-callback
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
