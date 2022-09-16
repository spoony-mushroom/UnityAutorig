set(SOURCES 
    stdafx.cpp
    AttachWeights.cpp)

add_executable(AttachWeights ${SOURCES})

target_include_directories(AttachWeights PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(AttachWeights PRIVATE pinocchio)

if (LINUX)
    target_compile_definitions(AttachWeights PRIVATE LINUX)
elseif(APPLE)
    target_compile_definitions(AttachWeights PRIVATE OSX)
endif()