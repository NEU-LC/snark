SOURCE_GROUP( flycapture-cat FILES flycapture-cat.cpp )
ADD_EXECUTABLE( flycapture-cat flycapture-cat.cpp )
set_target_properties( flycapture-cat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
TARGET_LINK_LIBRARIES( flycapture-cat snark_flycapture snark_imaging flycapture ${Boost_PROGRAM_OPTIONS_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

SOURCE_GROUP( flycapture-multicam FILES flycapture-multicam.cpp )
ADD_EXECUTABLE( flycapture-multicam flycapture-multicam.cpp )
set_target_properties( flycapture-multicam PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
TARGET_LINK_LIBRARIES( flycapture-multicam snark_flycapture snark_imaging flycapture ${Boost_PROGRAM_OPTIONS_LIBRARY} ${snark_ALL_EXTERNAL_LIBRARIES} tbb )

INSTALL( TARGETS flycapture-cat flycapture-multicam 
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
