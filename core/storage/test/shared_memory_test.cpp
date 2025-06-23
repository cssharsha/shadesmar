#include "core/storage/shared_memory_wrapper.hpp"
#include <gtest/gtest.h>
#include <glog/logging.h>

using namespace core::storage;

TEST(SharedMemoryTest, BasicSharedMemory) {
    // Test creation
    SharedMemoryWrapper wrapper("test_map_store");
    ASSERT_TRUE(wrapper.initialize(100, 50, 10));
    
    // Test basic access
    auto* region = wrapper.getRegion();
    ASSERT_NE(region, nullptr);
    
    // Test header validation
    EXPECT_EQ(region->header.magic, SharedMapStoreHeader::MAGIC_NUMBER);
    
    // Test array access
    auto* kf_indices = wrapper.getKeyFrameIndices();
    auto* kp_indices = wrapper.getKeyPointIndices();
    auto* splat_indices = wrapper.getSplatIndices();
    
    ASSERT_NE(kf_indices, nullptr);
    ASSERT_NE(kp_indices, nullptr);
    ASSERT_NE(splat_indices, nullptr);
}

TEST(SharedMemoryTest, AtomicOperations) {
    SharedMemoryWrapper wrapper("test_atomic_ops");
    ASSERT_TRUE(wrapper.initialize());
    
    auto* region = wrapper.getRegion();
    
    // Test atomic increments
    region->header.total_keyframes.store(0);
    region->header.keyframe_index_version.store(0);
    
    for (int i = 0; i < 10; ++i) {
        region->header.total_keyframes.fetch_add(1);
        region->header.keyframe_index_version.fetch_add(1);
    }
    
    EXPECT_EQ(region->header.total_keyframes.load(), 10);
    EXPECT_EQ(region->header.keyframe_index_version.load(), 10);
}

TEST(SharedMemoryTest, MultipleProcesses) {
    // Create first instance
    SharedMemoryWrapper wrapper1("test_multi_process");
    ASSERT_TRUE(wrapper1.initialize());
    
    // Set some values
    auto* region1 = wrapper1.getRegion();
    region1->header.total_keyframes.store(42);
    region1->header.vslam_process_healthy.store(true);
    
    // Create second instance (should attach to existing)
    SharedMemoryWrapper wrapper2("test_multi_process");
    ASSERT_TRUE(wrapper2.initialize());
    
    // Verify second instance sees the same data
    auto* region2 = wrapper2.getRegion();
    EXPECT_EQ(region2->header.total_keyframes.load(), 42);
    EXPECT_TRUE(region2->header.vslam_process_healthy.load());
}

TEST(SharedMemoryTest, IndexOperations) {
    SharedMemoryWrapper wrapper("test_index_ops");
    ASSERT_TRUE(wrapper.initialize(10, 5, 3));
    
    auto* kf_indices = wrapper.getKeyFrameIndices();
    
    // Test keyframe index operations
    kf_indices[0].id = 1001;
    kf_indices[0].disk_offset = 12345;
    kf_indices[0].data_size = 567;
    kf_indices[0].timestamp_ns = 1234567890;
    kf_indices[0].is_optimized = true;
    kf_indices[0].is_valid = true;
    
    // Verify data
    EXPECT_EQ(kf_indices[0].id, 1001);
    EXPECT_EQ(kf_indices[0].disk_offset, 12345);
    EXPECT_EQ(kf_indices[0].data_size, 567);
    EXPECT_TRUE(kf_indices[0].is_optimized);
    EXPECT_TRUE(kf_indices[0].is_valid);
}