# Find SWIG
#find_package(SWIG REQUIRED)
#include(${SWIG_USE_FILE})


include_directories(${CMAKE_SOURCE_DIR}/agent_implementation)

add_library(agent_implementation SHARED
        agent.h
        agent.cpp
        coordinate.h
        coordinate.cpp
        radio.h
        radio.cpp
        quadtree.h
        quadtree.cpp
            )

# Make sure CMake can find the SWIG generated files
set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR})

# Create a SWIG library
#set_property(SOURCE agent_implementation.i PROPERTY CPLUSPLUS ON)
#swig_add_library(agent_implementation TYPE SHARED LANGUAGE lua SOURCES
#        agent_implementation.i
#        agent.h
#        agent.cpp
#        coordinate.h
#        coordinate.cpp
#        radio.h
#        radio.cpp
#        quadtree.h
#        quadtree.cpp)
#swig_link_libraries(agent_implementation ${LUA_LIBRARIES})