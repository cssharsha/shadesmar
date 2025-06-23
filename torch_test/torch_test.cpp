#include <torch/torch.h>
#include <iostream>
#include <vector>

int main() {
    std::cout << "Testing LibTorch integration..." << std::endl;
    
    // Test 1: Check if PyTorch is available
    std::cout << "PyTorch version: " << TORCH_VERSION_MAJOR << "." 
              << TORCH_VERSION_MINOR << "." << TORCH_VERSION_PATCH << std::endl;
    
    // Test 2: Create tensors
    torch::Tensor tensor1 = torch::rand({2, 3});
    torch::Tensor tensor2 = torch::ones({2, 3});
    
    std::cout << "Created random tensor (2x3):" << std::endl;
    std::cout << tensor1 << std::endl;
    
    std::cout << "Created ones tensor (2x3):" << std::endl;
    std::cout << tensor2 << std::endl;
    
    // Test 3: Basic operations
    torch::Tensor result = tensor1 + tensor2;
    std::cout << "Addition result:" << std::endl;
    std::cout << result << std::endl;
    
    // Test 4: Matrix multiplication
    torch::Tensor matrix1 = torch::rand({3, 4});
    torch::Tensor matrix2 = torch::rand({4, 2});
    torch::Tensor matmul_result = torch::matmul(matrix1, matrix2);
    
    std::cout << "Matrix multiplication (3x4) * (4x2) = (3x2):" << std::endl;
    std::cout << matmul_result << std::endl;
    
    // Test 5: Check CUDA availability with detailed info
    std::cout << "CUDA availability check:" << std::endl;
    std::cout << "torch::cuda::is_available(): " << torch::cuda::is_available() << std::endl;
    std::cout << "torch::cuda::device_count(): " << torch::cuda::device_count() << std::endl;
    
    if (torch::cuda::is_available()) {
        std::cout << "CUDA is available!" << std::endl;
        std::cout << "CUDA device count: " << torch::cuda::device_count() << std::endl;
        
        try {
            torch::Tensor cuda_tensor = torch::rand({2, 2}).to(torch::kCUDA);
            std::cout << "CUDA tensor:" << std::endl;
            std::cout << cuda_tensor << std::endl;
            
            // Test CUDA computation
            torch::Tensor cuda_result = cuda_tensor * 2.0;
            std::cout << "CUDA computation result:" << std::endl;
            std::cout << cuda_result << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "CUDA tensor creation failed: " << e.what() << std::endl;
        }
    } else {
        std::cout << "CUDA is not available" << std::endl;
    }
    
    // Test 6: Gradient computation
    torch::Tensor x = torch::rand({2, 2}, torch::requires_grad(true));
    torch::Tensor y = x.sum();
    y.backward();
    
    std::cout << "Gradient computation test:" << std::endl;
    std::cout << "x.grad():" << std::endl;
    std::cout << x.grad() << std::endl;
    
    std::cout << "LibTorch integration test completed successfully!" << std::endl;
    return 0;
}