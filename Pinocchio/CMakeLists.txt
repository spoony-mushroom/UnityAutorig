set(SOURCES 
    attachment.cpp
    discretization.cpp
    embedding.cpp
    graphutils.cpp
    indexer.cpp
    intersector.cpp
    lsqSolver.cpp
    matrix.cpp
    mesh.cpp
    pinocchioApi.cpp
    refinement.cpp
    skeleton.cpp)

if (WIN32)
    list(APPEND SOURCES Pinocchio.cpp)
endif()

add_library(pinocchio ${SOURCES})

target_include_directories(pinocchio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})