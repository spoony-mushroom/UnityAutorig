cmake_minimum_required(VERSION 3.21)

project(Pinocchio)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(Pinocchio)
add_subdirectory(UnityPlugin)

option(BUILD_CLI "Build AttachWeights exectuable")
option(BUILD_DEMO "Build demo UI (requires FLTK)")

if (BUILD_DEMO)
    add_subdirectory(DemoUI)
endif()

if (BUILD_CLI)
    add_subdirectory(AttachWeights)
endif()