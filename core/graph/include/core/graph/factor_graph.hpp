#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <core/types/factor.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <core/types/imu.hpp>
#include "core/storage/map_store.hpp"

namespace core {
namespace graph {

using KeyFramePtr = types::KeyFrame::Ptr;

class FactorGraph {
public:
    FactorGraph() = default;

    void addKeyFrame(const KeyFramePtr& keyframe);
    void addFactor(const types::Factor& factor);

    // Bundle adjustment support - add landmarks as variables
    void addLandmark(uint32_t landmark_id, const Eigen::Vector3d& position);
    void addProjectionFactor(uint64_t keyframe_id, uint32_t landmark_id,
                           const Eigen::Vector2d& measurement,
                           const gtsam::SharedNoiseModel& noise_model,
                           const gtsam::Cal3_S2& camera_calibration,
                           const gtsam::Pose3& body_to_camera_transform = gtsam::Pose3());

    bool optimize();

    // Memory management
    bool removeKeyFrame(uint64_t id);  // Remove keyframe from memory (but keep in storage)

    // On-demand optimization from storage with full bundle adjustment
    bool optimizeFromStorage(const core::storage::MapStore& store);
    bool optimizeFromStorageWithLandmarks(const core::storage::MapStore& store,
                                         const std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // Get optimization results
    std::map<uint64_t, types::Pose> getOptimizedPoses() const;
    std::map<uint32_t, Eigen::Vector3d> getOptimizedLandmarks() const;

    KeyFramePtr getKeyFramePtr(uint64_t id) const;
    types::KeyFrame& getKeyFrame(uint64_t id) const;
    std::vector<KeyFramePtr> getAllKeyFrames() const;
    std::vector<types::Factor> getFactors() const;

private:
    std::map<uint64_t, KeyFramePtr> keyframes_;
    std::vector<types::Factor> factors_;

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;
    gtsam::Values result_;

    void updateGTSAMGraph();
    gtsam::Pose3 toPose3(const types::Pose& pose) const;
    types::Pose fromPose3(const gtsam::Pose3& pose3) const;
    gtsam::Point3 toPoint3(const Eigen::Vector3d& point) const;
    Eigen::Vector3d fromPoint3(const gtsam::Point3& point3) const;
    gtsam::Cal3_S2 createCameraCalibration(const core::types::CameraInfo& camera_info) const;

    // IMU helper functions
    gtsam::Vector3 toVector3(const Eigen::Vector3d& vec) const;
    gtsam::Matrix3 toMatrix3(const Eigen::Matrix3d& mat) const;
};

}  // namespace graph
}  // namespace core
