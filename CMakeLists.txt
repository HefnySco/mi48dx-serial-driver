cmake_minimum_required(VERSION 3.10)
project(mi48_lib_c VERSION 2.1.0 LANGUAGES CXX)

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

# Define version
add_definitions(-D__APP__VERSION__="${PROJECT_VERSION}")


# Include directories
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

# Build options
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_SHARED_LIBS "Build shared library" ON)
option(BUILD_STATIC_LIBS "Build static library" ON)

# Compiler flags
add_compile_options(-Wall -fPIC)

# Source files
set(LIB_SOURCES 
    serial_mi48.cpp
    thermal_visualization.cpp
)
set(LIB_HEADERS 
    serial_mi48.hpp
    thermal_visualization.hpp
    version.hpp
)

# Create static library
if(BUILD_STATIC_LIBS)
    add_library(mi48_static STATIC ${LIB_SOURCES})
    target_include_directories(mi48_static PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(mi48_static ${OPENCV_LIBRARIES} ${SERIALPORT_LIB})
    target_compile_options(mi48_static PUBLIC ${OPENCV_CFLAGS_OTHER})
    set_target_properties(mi48_static PROPERTIES OUTPUT_NAME mi48)
endif()

# Create shared library
if(BUILD_SHARED_LIBS)
    add_library(mi48_shared SHARED ${LIB_SOURCES})
    target_include_directories(mi48_shared PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(mi48_shared ${OPENCV_LIBRARIES} ${SERIALPORT_LIB})
    target_compile_options(mi48_shared PUBLIC ${OPENCV_CFLAGS_OTHER})
    set_target_properties(mi48_shared PROPERTIES OUTPUT_NAME mi48)
endif()

# Example executables
set(EXAMPLE_SOURCES
    examples/simple_display.cpp
    examples/advanced_display.cpp
    examples/raw_display.cpp
    examples/dual_camera.cpp
    examples/enhanced_display.cpp
    examples/multi_palette_display.cpp
)

set(EXAMPLE_TARGETS
    simple_display
    advanced_display
    raw_display
    dual_camera
    enhanced_display
    multi_palette_display
)

# Create example executables
if(BUILD_EXAMPLES)
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
endif()

# Installation
set(INSTALL_TARGETS)
if(BUILD_STATIC_LIBS)
    list(APPEND INSTALL_TARGETS mi48_static)
endif()
if(BUILD_SHARED_LIBS)
    list(APPEND INSTALL_TARGETS mi48_shared)
endif()

if(INSTALL_TARGETS)
    install(TARGETS ${INSTALL_TARGETS}
        EXPORT mi48Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )
    
    # Export targets for find_package
    install(EXPORT mi48Targets
        FILE mi48Targets.cmake
        NAMESPACE mi48::
        DESTINATION lib/cmake/mi48
    )
endif()

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
set(LIB_DEPENDS)
if(BUILD_STATIC_LIBS)
    list(APPEND LIB_DEPENDS mi48_static)
endif()
if(BUILD_SHARED_LIBS)
    list(APPEND LIB_DEPENDS mi48_shared)
endif()

if(LIB_DEPENDS)
    add_custom_target(libraries
        DEPENDS ${LIB_DEPENDS}
        COMMENT "Build libraries"
    )
endif()

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
    COMMAND ${CMAKE_COMMAND} -E echo "  all                    - Build libraries and examples"
    COMMAND ${CMAKE_COMMAND} -E echo "  mi48_static            - Build static library only"
    COMMAND ${CMAKE_COMMAND} -E echo "  mi48_shared            - Build shared library only"
    COMMAND ${CMAKE_COMMAND} -E echo "  libraries              - Build both libraries"
    COMMAND ${CMAKE_COMMAND} -E echo "  enhanced_display       - Build enhanced thermal display example"
    COMMAND ${CMAKE_COMMAND} -E echo "  multi_palette_display  - Build multi-palette thermal display example"
    COMMAND ${CMAKE_COMMAND} -E echo "  simple_display         - Build simple thermal display example"
    COMMAND ${CMAKE_COMMAND} -E echo "  advanced_display       - Build advanced thermal display example"
    COMMAND ${CMAKE_COMMAND} -E echo "  raw_display            - Build raw thermal display example"
    COMMAND ${CMAKE_COMMAND} -E echo "  dual_camera            - Build dual camera example"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean                  - Remove build artifacts"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean_all              - Remove all build files"
    COMMAND ${CMAKE_COMMAND} -E echo "  install                - Install libraries and headers"
    COMMAND ${CMAKE_COMMAND} -E echo "  show_help              - Show this help message"
)
