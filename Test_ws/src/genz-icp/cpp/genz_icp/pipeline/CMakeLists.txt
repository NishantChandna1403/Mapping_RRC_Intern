# Find PCL (Point Cloud Library)
find_package(PCL REQUIRED COMPONENTS common io)

# Define the library target
add_library(pipeline STATIC)
add_library(genz_icp::pipeline ALIAS pipeline)

# Add source files
target_sources(pipeline PRIVATE GenZICP.cpp)

# Include PCL headers
target_include_directories(pipeline PUBLIC ${PCL_INCLUDE_DIRS})

# Link against PCL libraries
target_link_libraries(pipeline
  PUBLIC
    genz_icp::core
    ${PCL_LIBRARIES}
)

# Add any PCL-specific compile definitions
target_compile_definitions(pipeline PRIVATE ${PCL_DEFINITIONS})

# Optional: if you have a custom function
set_global_target_properties(pipeline)
