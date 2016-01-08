IF (UNIX)
    OPTION( snark_INSTALL_BASH_COMPLETION "Install the BASH completion scripts." ON )
    SET( snark_INSTALL_BASH_COMPLETION_DIR /etc/bash_completion.d
         CACHE PATH "Location in which to install the bash completion scripts.")
    IF( snark_INSTALL_BASH_COMPLETION )
        INSTALL(FILES snark DESTINATION ${snark_INSTALL_BASH_COMPLETION_DIR} )
    ENDIF()
ENDIF()

