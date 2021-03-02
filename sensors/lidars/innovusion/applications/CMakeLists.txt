string( REGEX MATCH "17" version_match ${CXX_STANDARD_TO_USE} )
if( "${version_match}" STREQUAL "" )
    message( FATAL_ERROR "\ninnovusion-cat requires C++17\n"
        "Set CXX_STANDARD_TO_USE (or disable this module)"
        )
endif()

source_group( innovusion-cat FILES innovusion-cat.cpp )
add_executable( innovusion-cat innovusion-cat.cpp )
set_target_properties( innovusion-cat PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE )
set_target_properties( innovusion-cat PROPERTIES LINK_FLAGS_RELEASE -s )
target_link_libraries( innovusion-cat snark_innovusion ${Innovusion_LIBRARIES} )

install( TARGETS innovusion-cat
         RUNTIME DESTINATION ${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
