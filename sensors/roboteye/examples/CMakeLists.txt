# CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
# PROJECT(ocular)

# # Here you can point to a specific RobotEye library location if needed
# # This directory must contain the RobotEyeConfig.cmake file
# # Just set RobotEye_DIR to the desired location before calling FIND_PACKAGE
# IF(WIN32)
#   if(${CMAKE_CL_64})
#     MESSAGE(STATUS "Architecture: x64")
#     set(RobotEye_DIR "C:/Program Files/RobotEye/cmake")
#   else()
#     MESSAGE(STATUS "Architecture: x86")
#     set(RobotEye_DIR "C:/Program Files (x86)/RobotEye/cmake")
#   endif(${CMAKE_CL_64})
# else()
#     set(RobotEye_DIR "/opt/RobotEye/cmake")
# endif(WIN32)
# FIND_PACKAGE( RobotEye REQUIRED )

# MESSAGE( STATUS "RobotEye_LIBRARIES: " ${RobotEye_LIBRARIES} )
# MESSAGE( STATUS "RobotEye_INCLUDE_DIR: " ${RobotEye_INCLUDE_DIR} )

# INCLUDE_DIRECTORIES( ${RobotEye_INCLUDE_DIR} )

# FILE(GLOB ALL_CPP_SRCS *.cpp)
# FILE(GLOB ALL_H_SRCS *.h)
# foreach(src ${ALL_CPP_SRCS})
#   get_filename_component(program_name ${src} NAME_WE)
#   add_executable(${program_name} ${src} ${ALL_H_SRCS})
#   target_link_libraries(${program_name} ${RobotEye_LIBRARIES})
# endforeach()

source_group( ocular-roboteye-grab FILES ocular-roboteye-grab.cpp )
add_executable( ocular-roboteye-grab ocular-roboteye-grab.cpp )
target_link_libraries( ocular-roboteye-grab ${RobotEye_LIBRARIES} )
install( TARGETS ocular-roboteye-grab
         RUNTIME DESTINATION bin
         COMPONENT Runtime )
