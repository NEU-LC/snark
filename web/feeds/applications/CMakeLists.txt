INSTALL( PROGRAMS
            snark-frames
            snark-webframes
            tcp-ws-server
         DESTINATION ${CMAKE_INSTALL_PREFIX}/${snark_INSTALL_BIN_DIR}
         COMPONENT Runtime )
