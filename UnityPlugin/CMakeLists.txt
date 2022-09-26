set(SOURCES plugin.cpp)

add_library(UnityAutorig SHARED ${SOURCES})
target_link_libraries(UnityAutorig PRIVATE pinocchio)