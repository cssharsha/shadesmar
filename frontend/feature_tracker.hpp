#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "core/graph/factor_graph.hpp"
#include "core/types/keyframe.hpp"
#include "types.hpp"

namespace frontend {
namespace camera {

class FeatureTracker {
public:
    FeatureTracker(FeatureType type = FeatureType::ORB,
                   std::shared_ptr<core::graph::FactorGraph>& graph = nullptr);
    void processKeyframe();
    void extractFeatures(core::types::KeyFrame& keyframe);
    void trackIntraKeyframe(core::types::KeyFrame& keyframe);
    void trackInterKeyframe(core::types::KeyFrame& prev_keyframe,
                            core::types::KeyFrame& curr_keyframe);

private:
    cv::Ptr<cv::Feature2D> detector_;
    std::shared_ptr<core::graph::FactorGraph> graph_;
    FeatureType feature_type_;
    std::unordered_map<uint64_t, Track> tracks_;
    uint64_t next_track_;

    uint64_t num_keyframes_processed_{0U};
};
}  // namespace camera

}  // namespace frontend
