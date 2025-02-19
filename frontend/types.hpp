#include <cstdint>
#include <vector>

#include <opencv2/core/types.hpp>

#include "core/types/pose.hpp"

namespace frontend {
namespace camera {

static constexpr uint32_t MAX_FEATURES = 2000;

enum class FeatureType { ORB, FAST, GFTT };

struct Track {
    std::vector<cv::Point3f> positions;
    std::vector<uint64_t> frame_ids;
    std::vector<uint64_t> keyframe_ids;
    core::types::Pose landmark;
    bool is_triangulated = false;
    uint64_t track_id;
};

}  // namespace camera

}  // namespace frontend
