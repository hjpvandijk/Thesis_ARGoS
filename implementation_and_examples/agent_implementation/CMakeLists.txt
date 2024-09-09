# Include directories at the top-level include paths
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/libraries)  # Parent directory of cereal

# Define the library
add_library(agent_implementation SHARED
        agent.h
        agent.cpp
        coordinate.h
        coordinate.cpp
        radio.h
        radio.cpp
        Quadtree.h
        Box.h
)

# Link directories (if you have specific libraries to link against)
link_directories(${CMAKE_BINARY_DIR}/libraries/cereal)
