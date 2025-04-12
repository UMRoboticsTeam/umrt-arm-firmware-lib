# Copy any dependencies here
find_dependency(Boost REQUIRED COMPONENTS thread)
find_dependency(openFrameworksArduino CONFIG REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/@PROJECT_NAME@Targets.cmake)