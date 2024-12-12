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
        agent_control/path_planning/WallFollower.h
        agent_control/path_planning/WallFollower.cpp
        agent_control/path_planning/SimplePathPlanner.h
        agent_control/path_planning/SimplePathPlanner.cpp
        agent_control/path_planning/PathFollower.h
        agent_control/path_planning/PathFollower.cpp
        utils/CustomComparator.h
        agent_control/path_planning/FrontierEvaluator.h
        agent_control/path_planning/FrontierEvaluator.cpp
        agent_control/path_planning/ForceVectorCalculator.h
        agent_control/path_planning/ForceVectorCalculator.cpp
        agent_control/battery/BatteryManager.h
        agent_control/battery/BatteryManager.cpp
        agent_control/battery/MicroControllerBatteryManager.h
        agent_control/battery/MicroControllerBatterManager.cpp
        agent_control/battery/MotionSystemBatteryManager.h
        agent_control/battery/MotionSystemBatteryManager.cpp
)

# Link directories (if you have specific libraries to link against)
link_directories(${CMAKE_BINARY_DIR}/libraries/cereal)
