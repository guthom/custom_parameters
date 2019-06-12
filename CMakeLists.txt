cmake_minimum_required(VERSION 2.8.3)
project(custom_parameter)

## Find catkin macros and libraries.
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy
    )

#include directories
include_directories(include ${CATKIN_DEVEL_PREFIX}/include)
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare the C++ library source files
set(LIB_SOURCES
    src/parameter/parameter.cpp
    src/parameter/parameterHandler.cpp
    )

## Declare the external headers to show them in the project tree
set(EXTERNAL_HEADERS
    include/${PROJECT_NAME}/parameter.h
    include/${PROJECT_NAME}/parameterHandler.h
    )

##Add the libraries and external header files
add_library(${PROJECT_NAME} ${LIB_SOURCES} ${EXTERNAL_HEADERS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_package(CATKIN_DEPENDS roscpp
               #DEPENDS
               INCLUDE_DIRS include/
               LIBRARIES ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
