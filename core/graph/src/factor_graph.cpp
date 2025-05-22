#include "core/graph/factor_graph.hpp"

namespace core {
namespace graph {

void FactorGraph::addKeyFrame(const KeyFramePtr& keyframe) {
    keyframes_[keyframe->id] = keyframe;

    // Add to GTSAM initial estimates
    initial_estimates_.insert(gtsam::Symbol('x', keyframe->id), toPose3(keyframe->pose));
}

void FactorGraph::addFactor(const types::Factor& factor) {
    factors_.push_back(factor);

    switch (factor.type) {
        case proto::FactorType::PRIOR: {
            const auto& pose = std::get<0>(factor.measurement);
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                gtsam::Symbol('x', factor.connected_nodes[0]), toPose3(pose),
                gtsam::noiseModel::Gaussian::Information(factor.information)));
            break;
        }
        case proto::FactorType::ODOMETRY:
        case proto::FactorType::LOOP_CLOSURE: {
            const auto& relative_pose = std::get<0>(factor.measurement);
            graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', factor.connected_nodes[0]),
                gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                gtsam::noiseModel::Gaussian::Information(factor.information)));
            break;
        }
        default:
            throw std::runtime_error("Unsupported factor type");
    }
}

bool FactorGraph::optimize() {
    try {
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");
        params.setMaxIterations(100);
        params.setRelativeErrorTol(1e-5);

        gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimates_, params);
        result_ = optimizer.optimize();

        // Update keyframe poses with optimized results
        for (auto& [id, keyframe] : keyframes_) {
            const gtsam::Symbol symbol('x', id);
            if (result_.exists(symbol)) {
                keyframe->pose = fromPose3(result_.at<gtsam::Pose3>(symbol));
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Optimization failed: " << e.what() << std::endl;
        return false;
    }
}

gtsam::Pose3 FactorGraph::toPose3(const types::Pose& pose) const {
    return gtsam::Pose3(gtsam::Rot3(pose.orientation.matrix()), gtsam::Point3(pose.position));
}

types::Pose FactorGraph::fromPose3(const gtsam::Pose3& pose3) const {
    types::Pose pose;
    pose.position = pose3.translation();
    pose.orientation = Eigen::Quaterniond(pose3.rotation().matrix());
    return pose;
}

KeyFramePtr FactorGraph::getKeyFramePtr(uint64_t id) const {
    auto it = keyframes_.find(id);
    return it != keyframes_.end() ? it->second : nullptr;
}

types::KeyFrame& FactorGraph::getKeyFrame(uint64_t id) const {
    auto it = keyframes_.find(id);
    return *(it->second);
}

std::vector<KeyFramePtr> FactorGraph::getAllKeyFrames() const {
    std::vector<KeyFramePtr> keyframes;
    keyframes.reserve(keyframes_.size());
    for (const auto& [_, keyframe] : keyframes_) {
        keyframes.push_back(keyframe);
    }
    return keyframes;
}

std::vector<types::Factor> FactorGraph::getFactors() const {
    return factors_;
}

}  // namespace graph
}  // namespace core
