include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(coverage_loop_functions_pipuck MODULE
        coverage_loop_functions_pipuck.h
        coverage_qtuser_functions_pipuck.h
        coverage_loop_functions_pipuck.cpp
        coverage_qtuser_functions_pipuck.cpp)

target_link_libraries(coverage_loop_functions_pipuck
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
