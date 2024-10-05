#include "cuda_utils.h"

__global__ void setNullptrKernel(int** ptr_array, int index) {
    if (threadIdx.x == 0 && blockIdx.x == 0) {
        ptr_array[index] = nullptr;
    }
}