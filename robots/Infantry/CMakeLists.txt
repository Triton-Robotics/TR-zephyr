cmake_minimum_required(VERSION 3.20.0)

set(CONF_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../robots.conf"
    "${CMAKE_CURRENT_SOURCE_DIR}/prj.conf"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Infantry)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core core_build)

target_sources(app PRIVATE src/infantry.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../core)

# Otherwise has trouble getting core since it's up a level on the directory
target_link_libraries(app PRIVATE core)