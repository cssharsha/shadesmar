#pragma once

#include "shared_map_structs.hpp"
#include <string>
#include <memory>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

namespace core {
namespace storage {

class SharedMemoryWrapper {
public:
    explicit SharedMemoryWrapper(const std::string& name);
    ~SharedMemoryWrapper();
    
    // Create or attach to shared memory region
    bool initialize(uint32_t max_keyframes = 10000, 
                   uint32_t max_keypoints = 1000, 
                   uint32_t max_splats = 100);
    
    // Get pointer to shared region
    SharedMapStoreRegion* getRegion() { return region_; }
    const SharedMapStoreRegion* getRegion() const { return region_; }
    
    // Helper methods to access variable-length arrays
    SharedKeyFrameIndex* getKeyFrameIndices();
    SharedKeyPointIndex* getKeyPointIndices();
    SharedSplatIndex* getSplatIndices();
    
    // Check if shared memory is valid
    bool isValid() const { return region_ != nullptr; }
    
private:
    std::string shm_name_;
    int shm_fd_;
    size_t shm_size_;
    SharedMapStoreRegion* region_;
    bool is_creator_;
    
    bool createSharedMemory(size_t size);
    bool attachToSharedMemory();
    void cleanup();
};

} // namespace storage
} // namespace core