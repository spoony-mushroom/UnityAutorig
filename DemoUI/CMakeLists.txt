set(SOURCES 
    MyWindow.cpp
    defmesh.cpp
    demoUI.cpp
    filter.cpp
    motion.cpp
    processor.cpp)

add_executable(DemoUI ${SOURCES})

find_package(FLTK REQUIRED)
target_include_directories(DemoUI PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${FLTK_INCLUDE_DIR})
target_link_libraries(DemoUI PRIVATE pinocchio ${FLTK_LIBRARIES})
target_compile_definitions(DemoUI PRIVATE GL_SILENCE_DEPRECATION)