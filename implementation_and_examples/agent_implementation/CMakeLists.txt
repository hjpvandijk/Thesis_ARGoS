# Include directories at the top-level include paths
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/libraries)  # Parent directory of cereal

# Define the library
add_library(agent_implementation SHARED
        agent.h
        agent.cpp
        config.yaml
        coordinate.h
        coordinate.cpp
        radio.h
        radio.cpp
        Quadtree.h
        Box.h
        PheromoneMatrix.cpp
        PheromoneMatrix.h
        agent_control/sensing/simulation/distance_sensor/hc_sr04.h
        agent_control/sensing/simulation/distance_sensor/hc_sr04.cpp
        agent_control/motion/simulation/DifferentialDrive.h
        agent_control/motion/simulation/DifferentialDrive.cpp
        agent_control/battery/BatteryManager.h
        agent_control/battery/BatteryManager.cpp
        agent_control/battery/MicroControllerBatteryManager.h
        agent_control/battery/MicroControllerBatteryManager.cpp
        agent_control/battery/MotionSystemBatteryManager.h
        agent_control/battery/MotionSystemBatteryManager.cpp
)

# Link directories (if you have specific libraries to link against)
include(FetchContent)

FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG master# Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

target_link_libraries(agent_implementation PUBLIC yaml-cpp::yaml-cpp) # The library or executable that require yaml-cpp library
