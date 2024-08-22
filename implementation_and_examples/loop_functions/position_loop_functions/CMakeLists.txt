link_directories(${CMAKE_BINARY_DIR}/controllers/pipuck_hugo ${CMAKE_BINARY_DIR}/agent_implementation)

set(position_loop_functions_SOURCES position_loop_functions.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(position_loop_functions_SOURCES
          ${position_loop_functions_SOURCES}
          #    position_qt_user_functions.cpp
  )
endif(ARGOS_QTOPENGL_FOUND)

add_library(position_loop_functions MODULE ${position_loop_functions_SOURCES})
target_link_libraries(position_loop_functions
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_pipuck
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        agent_implementation)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(position_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)

include_directories(${CMAKE_SOURCE_DIR}/agent_implementation)