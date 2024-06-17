add_library(pipuck_hugo MODULE pipuck_hugo.h pipuck_hugo.cpp)
target_link_libraries(pipuck_hugo
        argos3core_simulator
        argos3plugin_simulator_pipuck
        argos3plugin_simulator_genericrobot)
