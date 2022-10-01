set(SOURCES plugin.cpp)

add_library(UnityAutoRig MODULE ${SOURCES})

if (APPLE)
    message("Target is macOS bundle")
    set_target_properties(UnityAutoRig PROPERTIES BUNDLE TRUE)
endif()

target_link_libraries(UnityAutoRig PRIVATE pinocchio)

# TODO: Do not export global symbols in libpinocchio from shared library

set(TEST_PLUGIN_DIR ${CMAKE_SOURCE_DIR}/SampleProject/Assets/Plugins)
add_custom_command(TARGET UnityAutoRig POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_BUNDLE_DIR:UnityAutoRig> ${TEST_PLUGIN_DIR}/$<TARGET_BUNDLE_DIR_NAME:UnityAutoRig>)