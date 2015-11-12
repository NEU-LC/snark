SOURCE_GROUP( flycapture-cat FILES flycapture-cat.cpp )
ADD_EXECUTABLE( flycapture-cat flycapture-cat.cpp )
TARGET_LINK_LIBRARIES( flycapture-cat snark_flycapture snark_imaging flycapture ${Boost_PROGRAM_OPTIONS_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

SOURCE_GROUP( flycapture-callback FILES flycapture-callback.cpp )
ADD_EXECUTABLE( flycapture-callback flycapture-callback.cpp )
TARGET_LINK_LIBRARIES( flycapture-callback snark_flycapture snark_imaging flycapture ${Boost_PROGRAM_OPTIONS_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

INSTALL( TARGETS flycapture-cat flycapture-callback
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
