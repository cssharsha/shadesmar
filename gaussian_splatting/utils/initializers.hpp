#pragma once

#include <core/storage/map_store.hpp>
#include <core/types/gaussian_splat.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>

#include "gaussian_splatting/training/gaussian_tensors.hpp"
#include "gaussian_splatting/training/keyframe_batch.hpp"
#include "gaussian_splatting/utils/point_cloud_utils.hpp"
// #include "torch_utils.hpp"

#include <cstdint>
#include <memory>

namespace gaussian_splatting {

bool intializeSplatsFromKeypoints(std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  std::vector<uint64_t>& keyframe_ids);
bool intializeSplatsFromKeypoints(const std::vector<uint64_t>& keyframe_ids,
                                  std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils);
bool intializeSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils);
bool intializeSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                  std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils);
bool initializeRandomSplats(const std::vector<uint64_t>& keyframe_ids,
                            std::shared_ptr<core::storage::MapStore>& map_store,
                            const uint64_t& current_batch_id, double& current_timestamp,
                            core::types::GaussianSplatBatch& splat_batch,
                            std::atomic<uint64_t>& next_splat_id, int splat_count);
Eigen::Vector3f extractColorFromKeyframes(const core::types::Keypoint& keypoint,
                                          std::shared_ptr<core::storage::MapStore>& map_store);
Eigen::Matrix3d computeKeypointCovariance(const core::types::Keypoint& keypoint,
                                          std::shared_ptr<core::storage::MapStore>& map_store);
Eigen::Matrix3d computeKeypointCovarianceFromScale(
    const core::types::Keypoint& keypoint, const std::vector<core::types::Keypoint>& all_keypoints,
    int k_neighbors = 10);
Eigen::Matrix3d computeKeypointCovarianceUsingScale(const core::types::Keypoint& keypoint,
                                                    const Eigen::Vector3f& scale);
Eigen::Matrix3d scaleToCovariance(const Eigen::Vector3f& scale,
                                  const Eigen::Quaternionf& direction);
bool testScaleCovarianceRoundtrip();
float computeInitialOpacity(const core::types::Keypoint& keypoint);
float computeInitialConfidence(const core::types::Keypoint& keypoint);
bool loadKeyframesToTensorBatch(const std::vector<uint64_t>& keyframe_ids,
                                const std::shared_ptr<core::storage::MapStore>& map_store,
                                const std::shared_ptr<stf::TransformTree>& tf_tree,
                                const torch::Device& device, const uint32_t& batch_id,
                                training::KeyframeBatch& keyframe_batch);
bool extractCameraPoses(const std::vector<uint64_t>& keyframe_ids,
                        const std::shared_ptr<core::storage::MapStore>& map_store,
                        const std::shared_ptr<stf::TransformTree>& tf_tree,
                        training::KeyframeBatch& keyframe_batch);
bool extractImageTensor(const core::types::KeyFrame::Ptr& keyframe, const torch::Device& device,
                        torch::Tensor& image_tensor, core::types::CameraInfo& camera_info);
torch::Tensor convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info,
                                              const torch::Device& device);
torch::Tensor convertCameraPoseToTensor(const Eigen::Isometry3d& pose, const torch::Device& device);
GaussianTensors convertSplatBatchToTensors(const core::types::GaussianSplatBatch& splat_batch,
                                           const torch::Device& device);

// Helper functions for random splat generation
std::pair<Eigen::Vector3f, Eigen::Vector3f> estimateSceneBoundsFromTrajectory(
    std::shared_ptr<core::storage::MapStore>& map_store);
std::vector<core::types::GaussianSplat> generateRandomSplats(const Eigen::Vector3f& scene_min,
                                                             const Eigen::Vector3f& scene_max,
                                                             int count,
                                                             std::atomic<uint64_t>& next_splat_id,
                                                             double timestamp);
Eigen::Vector3f generateRandomPosition(const Eigen::Vector3f& min_bounds,
                                       const Eigen::Vector3f& max_bounds);
Eigen::Vector3f generateRandomColor();
Eigen::Matrix3d generateInitialCovariance();
float generateInitialOpacity();

// Spatial partitioning utilities
core::types::GaussianSplatBatch filterSplatsByBoundingBox(
    const core::types::GaussianSplatBatch& splat_batch,
    const utils::BoundingBox& bbox);

std::vector<uint64_t> findKeyframesViewingRegion(
    const std::vector<core::types::KeyFrame::Ptr>& all_keyframes,
    const utils::BoundingBox& bbox,
    float viewing_distance_threshold = 50.0f);

bool isRegionVisibleFromKeyframe(
    const core::types::KeyFrame::Ptr& keyframe,
    const utils::BoundingBox& bbox,
    float viewing_distance_threshold = 50.0f);

// Keypoint outlier filtering
std::vector<core::types::Keypoint> filterKeypointsByDistanceFromCenter(
    const std::vector<core::types::Keypoint>& keypoints,
    const Eigen::Vector3f& center,
    float max_distance = 30.0f);

std::vector<core::types::Keypoint> filterSparseKeypoints(
    const std::vector<core::types::Keypoint>& keypoints,
    utils::PointCloudUtils& point_cloud_utils,
    int k_neighbors = 10,
    float density_threshold = 1.0f);

// Keyframe-based spatial partitioning
struct KeyframeRegion {
    utils::BoundingBox bbox_3d;  // 3D bounding box for this region
    std::vector<uint64_t> keyframe_ids;  // Keyframes in this region
    Eigen::Vector2f center_2d;  // 2D center in the dominant plane
    int region_id;
};

// Detect dominant 2D plane from keyframe positions (returns primary and secondary axes)
// Returns: pair of (primary_axis, secondary_axis) where each is 0=X, 1=Y, 2=Z
std::pair<int, int> detectDominant2DPlane(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes);

// Partition keyframes into 2D grid regions along the dominant plane
std::vector<KeyframeRegion> partitionKeyframesInto2DGrid(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes,
    const std::vector<core::types::Keypoint>& keypoints,
    int grid_rows = 3,
    int grid_cols = 3);

// Partition keyframes into radial sectors around a shared center (for outside-in viewing)
std::vector<KeyframeRegion> partitionKeyframesIntoRadialSectors(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes,
    const std::vector<core::types::Keypoint>& keypoints,
    int num_sectors = 8,
    float overlap_angle = 30.0f);  // Overlap in degrees

}  // namespace gaussian_splatting
