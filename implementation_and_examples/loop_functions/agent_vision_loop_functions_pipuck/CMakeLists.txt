include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(agent_vision_loop_functions_pipuck MODULE
        agent_vision_loop_functions_pipuck.h
        agent_vision_qtuser_functions_pipuck.h
        agent_vision_loop_functions_pipuck.cpp
        agent_vision_qtuser_functions_pipuck.cpp)

target_link_libraries(agent_vision_loop_functions_pipuck
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
