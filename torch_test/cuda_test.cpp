#include <cuda_runtime.h>
#include <iostream>

int main() {
    std::cout << "=== CUDA Runtime Test ===" << std::endl;
    
    // Check CUDA runtime version
    int runtime_version;
    cudaError_t error = cudaRuntimeGetVersion(&runtime_version);
    if (error != cudaSuccess) {
        std::cout << "CUDA Runtime error: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    std::cout << "CUDA Runtime version: " << runtime_version << std::endl;
    
    // Check driver version
    int driver_version;
    error = cudaDriverGetVersion(&driver_version);
    if (error != cudaSuccess) {
        std::cout << "CUDA Driver error: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    std::cout << "CUDA Driver version: " << driver_version << std::endl;
    
    // Get device count
    int device_count;
    error = cudaGetDeviceCount(&device_count);
    if (error != cudaSuccess) {
        std::cout << "CUDA Get Device Count error: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    std::cout << "CUDA Device count: " << device_count << std::endl;
    
    if (device_count == 0) {
        std::cout << "No CUDA devices found!" << std::endl;
        return 1;
    }
    
    // Get device properties
    for (int i = 0; i < device_count; i++) {
        cudaDeviceProp prop;
        error = cudaGetDeviceProperties(&prop, i);
        if (error != cudaSuccess) {
            std::cout << "Error getting device " << i << " properties: " << cudaGetErrorString(error) << std::endl;
            continue;
        }
        
        std::cout << "Device " << i << ": " << prop.name << std::endl;
        std::cout << "  Compute capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "  Global memory: " << prop.totalGlobalMem / (1024*1024) << " MB" << std::endl;
    }
    
    // Try to allocate memory on GPU
    float* d_ptr;
    error = cudaMalloc(&d_ptr, 1024 * sizeof(float));
    if (error != cudaSuccess) {
        std::cout << "CUDA malloc error: " << cudaGetErrorString(error) << std::endl;
        return 1;
    }
    std::cout << "CUDA memory allocation test: SUCCESS" << std::endl;
    
    cudaFree(d_ptr);
    std::cout << "=== CUDA test completed successfully ===" << std::endl;
    return 0;
}