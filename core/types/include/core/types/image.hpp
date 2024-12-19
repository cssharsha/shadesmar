#pragma once

#include <opencv2/core.hpp>
#include "core/proto/sensor_data.pb.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace core {
namespace types {

class Image {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cv::Mat data;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    std::string encoding;

    proto::Image toProto() const {
        proto::Image image_proto;
        image_proto.set_data(data.data, data.total() * data.elemSize());
        image_proto.set_width(width);
        image_proto.set_height(height);
        image_proto.set_channels(channels);
        image_proto.set_encoding(encoding);
        return image_proto;
    }

    static Image fromProto(const proto::Image& image_proto) {
        Image image;
        image.width = image_proto.width();
        image.height = image_proto.height();
        image.channels = image_proto.channels();
        image.encoding = image_proto.encoding();

        int cv_type;
        if (image.encoding == "32FC1") {
            cv_type = CV_32FC1;
        } else if (image.encoding == "bgr8" || image.encoding == "rgb8") {
            cv_type = CV_8UC3;
        } else if (image.encoding == "mono8") {
            cv_type = CV_8UC1;
        } else if (image.encoding == "16UC1") {
            cv_type = CV_16UC1;
        } else {
            // Default to 8UC with the specified number of channels
            cv_type = CV_8UC(image.channels);
        }

        cv::Mat mat(image.height, image.width, cv_type,
                    const_cast<void*>(static_cast<const void*>(image_proto.data().data())));
        image.data = mat.clone();

        return image;
    }

    static Image fromCvMat(const cv::Mat& mat, const std::string& encoding = "rgb8") {
        Image image;
        image.data = mat.clone();
        image.width = mat.cols;
        image.height = mat.rows;
        image.channels = mat.channels();
        image.encoding = encoding;
        return image;
    }

    cv::Mat toCvMat() const {
        return data;
    }
};

}  // namespace types
}  // namespace core
