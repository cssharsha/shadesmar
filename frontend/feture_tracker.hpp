#include <opencv2/opencv.hpp>

namespace frontend {
namespace camera {

static constexpr float32_t MAX_FEATURES = 2000;

enum class FeatureType { ORB, GFTT, FAST };

class FeatureTracker {
public:
    FeatureTracker(FeatureType type = FeatureType::ORB) : feature_type_(type) {
        switch (feature_type_) {
            case FeatureType::ORB:
                detector_ = cv::ORB::create(MAX_FEATURES);
                break;
            case FeatureType::GFTT:
                detector_ = cv::GFTTDetector::create(MAX_FEATURES, 0.01, 10, 3, true);
                break;
            case FeatureType::FAST:
                detector_ =
                    cv::FastFeatureDetector::create(20, true, cv::FastFeatureDetector::TYPE_9_16);
                break;
        }
    }

private:
    std::unique_ptr<cv::Feature2D> detector_;
    FeatureType feature_type_;
};
}  // namespace camera

}  // namespace frontend
