#ifndef CUDA_UTILS_H
#define CUDA_UTILS_H

#ifdef HAS_CUDA
#include <cuda_runtime.h>

#define CUDA_CHECK_ERROR(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << " at line " << __LINE__ << ": " << cudaGetErrorString(err) << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    } while (0)

__global__ void setNullptrKernel(int** ptr_array, int index);
#endif

#endif // CUDA_UTILS_H