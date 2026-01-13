cmake_minimum_required(VERSION 3.10)
project(mi48_lib_c VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
find_library(SERIALPORT_LIB NAMES serialport REQUIRED)

SET (__APP__VERSION__ "0.0.1")
add_definitions( -D__APP__VERSION__="${__APP__VERSION__}")
project(mi48_lib_c VERSION ${__APP__VERSION__})


# Include directories
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Compiler flags
add_compile_options(-Wall -fPIC)

# Source files
set(LIB_SOURCES serial_mi48.cpp)
set(LIB_HEADERS serial_mi48.hpp)

# Create static library
add_library(mi48_static STATIC ${LIB_SOURCES})
target_include_directories(mi48_static PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mi48_static ${OPENCV_LIBRARIES} ${SERIALPORT_LIB})
target_compile_options(mi48_static PUBLIC ${OPENCV_CFLAGS_OTHER})

# Create shared library
add_library(mi48_shared SHARED ${LIB_SOURCES})
target_include_directories(mi48_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mi48_shared ${OPENCV_LIBRARIES} ${SERIALPORT_LIB})
target_compile_options(mi48_shared PUBLIC ${OPENCV_CFLAGS_OTHER})

# Set library output names
set_target_properties(mi48_static PROPERTIES OUTPUT_NAME mi48)
set_target_properties(mi48_shared PROPERTIES OUTPUT_NAME mi48)

# Example executables
set(EXAMPLE_SOURCES
    examples/simple_display.cpp
    examples/advanced_display.cpp
    examples/raw_display.cpp
    examples/dual_camera.cpp
)

set(EXAMPLE_TARGETS
    simple_display
    advanced_display
    raw_display
    dual_camera
)

# Create example executables
list(LENGTH EXAMPLE_SOURCES EXAMPLE_COUNT)
math(EXPR EXAMPLE_COUNT_MINUS_1 "${EXAMPLE_COUNT} - 1")

foreach(i RANGE 0 ${EXAMPLE_COUNT_MINUS_1})
    list(GET EXAMPLE_SOURCES ${i} EXAMPLE_SRC)
    list(GET EXAMPLE_TARGETS ${i} EXAMPLE_TARGET)
    
    add_executable(${EXAMPLE_TARGET} ${LIB_SOURCES} ${EXAMPLE_SRC})
    target_include_directories(${EXAMPLE_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${EXAMPLE_TARGET} ${OPENCV_LIBRARIES} ${SERIALPORT_LIB})
    target_compile_options(${EXAMPLE_TARGET} PRIVATE ${OPENCV_CFLAGS_OTHER})
    
    # Add pthread for dual_camera
    if(${EXAMPLE_TARGET} STREQUAL "dual_camera")
        target_link_libraries(${EXAMPLE_TARGET} pthread)
    endif()
endforeach()

# Installation
install(TARGETS mi48_static mi48_shared
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${LIB_HEADERS} DESTINATION include)

# Create config file for finding the library
include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/mi48Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/mi48Config.cmake"
    INSTALL_DESTINATION lib/cmake/mi48
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mi48ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mi48Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mi48ConfigVersion.cmake"
    DESTINATION lib/cmake/mi48
)

# Custom targets
add_custom_target(libraries
    DEPENDS mi48_static mi48_shared
    COMMENT "Build both static and shared libraries"
)

add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMENT "Clean all build files"
)

# Help target (use different name since 'help' is reserved)
add_custom_target(show_help)
add_custom_command(TARGET show_help POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Available targets:"
    COMMAND ${CMAKE_COMMAND} -E echo "  all          - Build libraries and examples"
    COMMAND ${CMAKE_COMMAND} -E echo "  mi48_static  - Build static library only"
    COMMAND ${CMAKE_COMMAND} -E echo "  mi48_shared  - Build shared library only"
    COMMAND ${CMAKE_COMMAND} -E echo "  libraries    - Build both libraries"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean        - Remove build artifacts"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean_all    - Remove all build files"
    COMMAND ${CMAKE_COMMAND} -E echo "  install      - Install libraries and headers"
    COMMAND ${CMAKE_COMMAND} -E echo "  show_help    - Show this help message"
)
