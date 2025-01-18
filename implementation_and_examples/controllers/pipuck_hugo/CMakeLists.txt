link_directories(${CMAKE_BINARY_DIR}/agent_implementation)

add_library(pipuck_hugo MODULE pipuck_hugo.h pipuck_hugo.cpp error_heatmap.txt orientation_offset.txt position_direction_offset.txt)
target_link_libraries(pipuck_hugo
        argos3core_simulator
        argos3plugin_simulator_pipuck
        argos3plugin_simulator_genericrobot
        agent_implementation

)