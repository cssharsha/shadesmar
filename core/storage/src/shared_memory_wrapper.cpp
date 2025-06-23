#include "core/storage/shared_memory_wrapper.hpp"
#include <glog/logging.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

namespace core {
namespace storage {

SharedMemoryWrapper::SharedMemoryWrapper(const std::string& name) 
    : shm_name_("/" + name), shm_fd_(-1), shm_size_(0), region_(nullptr), is_creator_(false) {
}

SharedMemoryWrapper::~SharedMemoryWrapper() {
    cleanup();
}

bool SharedMemoryWrapper::initialize(uint32_t max_keyframes, uint32_t max_keypoints, uint32_t max_splats) {
    // Calculate required size
    shm_size_ = SharedMapStoreRegion::getTotalSize(max_keyframes, max_keypoints, max_splats);
    
    // Try to attach to existing shared memory first
    if (attachToSharedMemory()) {
        LOG(INFO) << "Attached to existing shared memory: " << shm_name_ << " (size: " << shm_size_ << ")";
        return true;
    }
    
    // If attachment failed, try to create new shared memory
    if (createSharedMemory(shm_size_)) {
        LOG(INFO) << "Created new shared memory: " << shm_name_ << " (size: " << shm_size_ << ")";
        
        // Initialize the header
        new (region_) SharedMapStoreHeader();
        region_->header.max_keyframes = max_keyframes;
        region_->header.max_keypoints_per_batch = max_keypoints;
        region_->header.max_splat_batches = max_splats;
        
        // Initialize arrays to zero
        memset(getKeyFrameIndices(), 0, max_keyframes * sizeof(SharedKeyFrameIndex));
        memset(getKeyPointIndices(), 0, max_keypoints * sizeof(SharedKeyPointIndex));
        memset(getSplatIndices(), 0, max_splats * sizeof(SharedSplatIndex));
        
        return true;
    }
    
    LOG(ERROR) << "Failed to initialize shared memory: " << shm_name_;
    return false;
}

bool SharedMemoryWrapper::createSharedMemory(size_t size) {
    // Create shared memory object
    shm_fd_ = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        LOG(ERROR) << "Failed to create shared memory: " << strerror(errno);
        return false;
    }
    
    // Set size
    if (ftruncate(shm_fd_, size) == -1) {
        LOG(ERROR) << "Failed to set shared memory size: " << strerror(errno);
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
        return false;
    }
    
    // Map to process address space
    region_ = static_cast<SharedMapStoreRegion*>(
        mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    
    if (region_ == MAP_FAILED) {
        LOG(ERROR) << "Failed to map shared memory: " << strerror(errno);
        close(shm_fd_);
        shm_unlink(shm_name_.c_str());
        region_ = nullptr;
        return false;
    }
    
    is_creator_ = true;
    return true;
}

bool SharedMemoryWrapper::attachToSharedMemory() {
    // Try to open existing shared memory
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0);
    if (shm_fd_ == -1) {
        return false; // Doesn't exist yet
    }
    
    // Get size
    struct stat shm_stat;
    if (fstat(shm_fd_, &shm_stat) == -1) {
        LOG(ERROR) << "Failed to get shared memory stats: " << strerror(errno);
        close(shm_fd_);
        return false;
    }
    
    // Verify size matches expected
    if (static_cast<size_t>(shm_stat.st_size) != shm_size_) {
        LOG(WARNING) << "Shared memory size mismatch. Expected: " << shm_size_ 
                     << ", Got: " << shm_stat.st_size;
        close(shm_fd_);
        return false;
    }
    
    // Map to process address space
    region_ = static_cast<SharedMapStoreRegion*>(
        mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
    
    if (region_ == MAP_FAILED) {
        LOG(ERROR) << "Failed to map shared memory: " << strerror(errno);
        close(shm_fd_);
        region_ = nullptr;
        return false;
    }
    
    // Validate magic number
    if (region_->header.magic != SharedMapStoreHeader::MAGIC_NUMBER) {
        LOG(ERROR) << "Invalid shared memory magic number";
        munmap(region_, shm_size_);
        close(shm_fd_);
        region_ = nullptr;
        return false;
    }
    
    is_creator_ = false;
    return true;
}

SharedKeyFrameIndex* SharedMemoryWrapper::getKeyFrameIndices() {
    if (!region_) return nullptr;
    return region_->keyframe_indices;
}

SharedKeyPointIndex* SharedMemoryWrapper::getKeyPointIndices() {
    if (!region_) return nullptr;
    char* base = reinterpret_cast<char*>(region_);
    size_t offset = SharedMapStoreRegion::getKeyPointIndicesOffset(region_->header.max_keyframes);
    return reinterpret_cast<SharedKeyPointIndex*>(base + offset);
}

SharedSplatIndex* SharedMemoryWrapper::getSplatIndices() {
    if (!region_) return nullptr;
    char* base = reinterpret_cast<char*>(region_);
    size_t offset = SharedMapStoreRegion::getSplatIndicesOffset(
        region_->header.max_keyframes, region_->header.max_keypoints_per_batch);
    return reinterpret_cast<SharedSplatIndex*>(base + offset);
}

void SharedMemoryWrapper::cleanup() {
    if (region_) {
        munmap(region_, shm_size_);
        region_ = nullptr;
    }
    
    if (shm_fd_ != -1) {
        close(shm_fd_);
        
        // Only unlink if we created it
        if (is_creator_) {
            shm_unlink(shm_name_.c_str());
        }
        
        shm_fd_ = -1;
    }
}

} // namespace storage
} // namespace core