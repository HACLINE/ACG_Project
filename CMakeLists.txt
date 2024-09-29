cmake_minimum_required(VERSION 3.10)
project(ACG_Project)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(main ${SOURCES})

target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/glm)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/tiny_obj_loader)

target_link_libraries(main yaml-cpp)