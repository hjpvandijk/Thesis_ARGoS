# Set the include directory for cereal as the parent of the cereal folder
include_directories(${CMAKE_SOURCE_DIR}/libraries)

# No need for add_subdirectory if cereal is header-only and does not have its own CMake setup
