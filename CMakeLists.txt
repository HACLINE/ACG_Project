cmake_minimum_required(VERSION 3.10)
project(ACG_Project)

set(CMAKE_CXX_STANDARD 17)

find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glm REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(main ${SOURCES})

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/glm)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/tiny_obj_loader)

target_link_libraries(main ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} glm::glm yaml-cpp::yaml-cpp)