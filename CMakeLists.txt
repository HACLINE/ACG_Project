cmake_minimum_required(VERSION 3.10)
project(ACG_Project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glm REQUIRED)
find_package(CUDAToolkit)

file(GLOB_RECURSE SOURCES "src/*.cpp")

if(CUDAToolkit_FOUND)
    add_definitions(-DHAS_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES 86)
    enable_language(CUDA)
    file(GLOB_RECURSE CUDA_SOURCES "src/*.cu")
    list(APPEND SOURCES ${CUDA_SOURCES})
endif()

add_executable(main ${SOURCES})

include_directories(${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/glm)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/tiny_obj_loader)
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/external/stb_image_write)

if(CUDAToolkit_FOUND)
    target_include_directories(main PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    target_link_libraries(main CUDA::cudart)
endif()

target_link_libraries(main ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} glm::glm yaml-cpp::yaml-cpp)
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)