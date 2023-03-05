cmake_minimum_required(VERSION 3.11)

project(graphs)

option(INSOURCEBUILD OFF)

if((${PROJECT_SOURCE_DIR} STREQUAL ${PROJECT_BINARY_DIR}) AND NOT ${INSOURCEBUILD})
  message(FATAL_ERROR "In-source building disabled. Provide -DINSOURCEBUILD option if you are sure about it.")
endif()

set(CMAKE_CXX_STANDARD 20)
include(FetchContent)

find_package(GTest)
set(NOGTEST OFF CACHE BOOL "Disable GoogleTest")

enable_testing()
if(NOT NOGTEST AND GTEST_FOUND AND CMAKE_MINOR_VERSION GREATER_EQUAL 20)
  set(ENABLE_GTEST ON)
else()
  message(WARNING "Google Test disabled")
endif()

set (GRAPHS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(DCMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(test)