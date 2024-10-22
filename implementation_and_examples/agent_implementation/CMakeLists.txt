# Include directories at the top-level include paths
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/libraries)  # Parent directory of cereal

# Define the library
add_library(agent_implementation SHARED
        agent.h
        agent.cpp
        utils/coordinate.h
        utils/coordinate.cpp
        agent_control/communication/simulation/radio.h
        agent_control/communication/simulation/radio.cpp
        utils/Quadtree.h
        utils/Box.h
        agent_control/sensing/simulation/distance_sensor/hc_sr04.h
        agent_control/sensing/simulation/distance_sensor/hc_sr04.cpp
        agent_control/motion/simulation/DifferentialDrive.h
        agent_control/motion/simulation/DifferentialDrive.cpp
)

# Link directories (if you have specific libraries to link against)
link_directories(${CMAKE_BINARY_DIR}/libraries/cereal)
