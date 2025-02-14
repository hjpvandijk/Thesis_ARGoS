
link_directories(${CMAKE_BINARY_DIR}/agent_implementation)

add_library(pipuck_hugo_lua MODULE pipuck_hugo.lua)


target_link_libraries(pipuck_hugo_lua
        argos3core_simulator
        argos3plugin_simulator_pipuck
        argos3plugin_simulator_genericrobot
        agent_implementation
)
