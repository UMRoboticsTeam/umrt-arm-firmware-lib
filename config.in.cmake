include(CMakeFindDependencyMacro)
# Copy any dependencies here
find_dependency(Boost REQUIRED COMPONENTS log log_setup filesystem)
find_dependency(openFrameworksArduino CONFIG REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake)