#add_subdirectory(archives)
#add_subdirectory(details)
#add_subdirectory(external)
#add_subdirectory(types)

include_directories(archives)
include_directories(details)
include_directories(external)
include_directories(types)

add_library(cereal SHARED
        access.hpp
        cereal.hpp
        macros.hpp
        specialize.hpp
        version.hpp
        archives
        details
        external
        types
)

