# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# compile CUDA with /usr/local/cuda/bin/nvcc
# compile CXX with /usr/bin/c++
CUDA_DEFINES = -DHAS_CUDA

CUDA_INCLUDES = --options-file CMakeFiles/main.dir/includes_CUDA.rsp

CUDA_FLAGS = -std=c++17 --generate-code=arch=compute_86,code=[compute_86,sm_86] -w

CXX_DEFINES = -DHAS_CUDA

CXX_INCLUDES = -I/root/ACG_Project/src -I/root/ACG_Project/external/glm -I/root/ACG_Project/external/tiny_obj_loader -I/root/ACG_Project/external/stb_image_write -I/root/ACG_Project/external/voxelizer -isystem /usr/include/opencv4 -isystem /usr/lib/cmake/glm -isystem /usr/local/cuda/include

CXX_FLAGS = -std=gnu++17 -w
