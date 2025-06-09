#pragma once

#include <opencv2/core.hpp>
#include "core/proto/sensor_data.pb.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace core {
namespace types {

struct CameraInfo {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<double> k;  // 3x3 camera matrix
    std::vector<double> d;  // distortion coefficients
    std::vector<double> p;  // projection matrix
    uint32_t width;
    uint32_t height;
    std::string distortion_model;
    std::string frame_id;

    proto::CameraInfo toProto() const {
        proto::CameraInfo proto_info;
        *proto_info.mutable_k() = {k.begin(), k.end()};
        *proto_info.mutable_d() = {d.begin(), d.end()};
        proto_info.set_width(width);
        proto_info.set_height(height);
        proto_info.set_distortion_model(distortion_model);
        proto_info.set_frame_id(frame_id);
        return proto_info;
    }

    static CameraInfo fromProto(const proto::CameraInfo& proto_info) {
        CameraInfo info;
        info.k = {proto_info.k().begin(), proto_info.k().end()};
        info.d = {proto_info.d().begin(), proto_info.d().end()};
        info.width = proto_info.width();
        info.height = proto_info.height();
        info.distortion_model = proto_info.distortion_model();
        info.frame_id = proto_info.frame_id();
        return info;
    }

    Eigen::Matrix3d getKInEigen() const {
        if (k.size() != 9) {
            throw std::runtime_error(
                "Input vector must contain exactly 9 elements for a 3x3 matrix.");
        }

        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> map_from_vector(k.data());

        Eigen::Matrix3d eigen_matrix = map_from_vector;

        return eigen_matrix;
    }
};

struct Image {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    cv::Mat data;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    std::string encoding;
    std::string frame_id;

    proto::Image toProto() const {
        proto::Image image_proto;
        image_proto.set_data(data.data, data.total() * data.elemSize());
        image_proto.set_width(width);
        image_proto.set_height(height);
        image_proto.set_channels(channels);
        image_proto.set_encoding(encoding);
        image_proto.set_frame_id(frame_id);
        return image_proto;
    }

    static Image fromProto(const proto::Image& image_proto) {
        Image image;
        image.width = image_proto.width();
        image.height = image_proto.height();
        image.channels = image_proto.channels();
        image.encoding = image_proto.encoding();
        image.frame_id = image_proto.frame_id();
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

    static Image fromCvMat(const cv::Mat& mat, const std::string& encoding = "rgb8",
                           const std::string& frame_id = "") {
        Image image;
        image.data = mat.clone();
        image.width = mat.cols;
        image.height = mat.rows;
        image.channels = mat.channels();
        image.encoding = encoding;
        image.frame_id = frame_id;
        return image;
    }

    cv::Mat toCvMat() const {
        return data;
    }
};

}  // namespace types
}  // namespace core
