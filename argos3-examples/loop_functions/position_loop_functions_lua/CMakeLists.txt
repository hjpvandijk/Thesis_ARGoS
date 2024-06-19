link_directories(${CMAKE_BINARY_DIR}/controllers/pipuck_hugo_lua ${CMAKE_BINARY_DIR}/agent_implementation)

set(position_loop_functions_lua_SOURCES position_loop_functions_lua.h position_loop_functions_lua.cpp)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(position_loop_functions_lua_SOURCES
          ${position_loop_functions_lua_SOURCES}
  )
endif(ARGOS_QTOPENGL_FOUND)

add_library(position_loop_functions_lua MODULE
        position_loop_functions_lua.h
        position_loop_functions_lua.cpp)
target_link_libraries(position_loop_functions_lua
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_pipuck
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_media
        agent_implementation)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(position_loop_functions_lua argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)

include_directories(${CMAKE_SOURCE_DIR}/agent_implementation)

