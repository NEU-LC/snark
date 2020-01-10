set( PROJECT "ipx" )
set( TARGET_NAME snark_${PROJECT} )

if( WIN32 )
    message( FATAL "not implemented" )
endif( WIN32 )

set( ARCH "x86_64" CACHE STRING "architecture of the project" )
if("${ARCH}" STREQUAL "x86")
    if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
        set(ARCH_DIR "Win32_i86")
        add_definitions(-D_WIN32)
    elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
        set(ARCH_DIR "Linux32_i86")
        add_definitions(-DLINUX)
    else()
        set(ARCH_DIR "x86")
    endif()
else()
    if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
        set(ARCH_DIR "Win64_x64")
        add_definitions(-D_WIN32)
    elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
        set(ARCH_DIR "Linux64_x64")
        add_definitions(-DLINUX)
    else()
        set(ARCH_DIR "x64")
    endif()
endif()

set( BINARY_DIR "${CMAKE_BINARY_DIR}/sensors/cameras/basler/applications" )
set( IPX_SDK_DIR "" CACHE PATH "ipx sdk directory" )

add_custom_target( ipx.conf ) # quick and dirty
add_custom_command( TARGET ipx.conf
                    POST_BUILD
                    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/ipx-generate-ld-conf ARGS "${IPX_SDK_DIR}/lib/${ARCH_DIR}" )
#install( FILES ${BINARY_DIR}/ipx.conf DESTINATION /etc/ld.so.conf.d )
install( CODE "message( \"sensors/cameras/ipx: running ldconfig\" )" )
install( CODE "execute_process( COMMAND ldconfig )" )

#add_dependencies( ipx-cat ipx.conf )

# target_link_libraries( snark_ipx
#                        ${snark_ALL_LIBRARIES}
#                        ${snark_ALL_EXTERNAL_LIBRARIES}
#                        tbb
#                        pylonbase
#                        pylonutility )

# voodoo, see pylon sample makefiles
#INCLUDE_DIRECTORIES( BEFORE SYSTEM ${BASLER_PYLON_DIR}/include )
# if building JAI is enabled, SYSTEM (that translates into c++ -isystem) makes compiler
# pick up JAI GenApi headers in some cases, which obviously does not compile
include_directories( ${IPX_SDK_DIR}/include )

# link_directories( ${IPX_SDK_DIR}/lib64 )

#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_GIGE -Wl,-E" )

file( GLOB source *.cpp )
file( GLOB includes *.h )
source_group( ${TARGET_NAME} FILES ${source} ${includes} )
#add_subdirectory( applications )
