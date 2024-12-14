#pragma once

#include <Eigen/Core>
#include <rerun/collection_adapter.hpp>

namespace rerun {

// Adapter for Eigen::Vector3f to Vec3D
template <>
struct CollectionAdapter<datatypes::Vec3D, Eigen::Matrix<float, 3, 1>> {
    auto operator()(const Eigen::Matrix<float, 3, 1>& vec) const {
        return datatypes::Vec3D{vec.x(), vec.y(), vec.z()};
    }
};

// Adapter for Eigen::Vector3d to Vec3D
template <>
struct CollectionAdapter<datatypes::Vec3D, Eigen::Matrix<double, 3, 1>> {
    auto operator()(const Eigen::Matrix<double, 3, 1>& vec) const {
        return datatypes::Vec3D{static_cast<float>(vec.x()), static_cast<float>(vec.y()),
                                static_cast<float>(vec.z())};
    }
};

// Add similar adapters for Position3D and Color if needed
template <>
struct CollectionAdapter<components::Position3D, Eigen::Matrix<float, 3, 1>> {
    auto operator()(const Eigen::Matrix<float, 3, 1>& vec) const {
        return components::Position3D{vec.x(), vec.y(), vec.z()};
    }
};

template <>
struct CollectionAdapter<components::Color, Eigen::Matrix<float, 3, 1>> {
    auto operator()(const Eigen::Matrix<float, 3, 1>& vec) const {
        return components::Color{vec.x(), vec.y(), vec.z()};
    }
};

// Add these specializations before any other code:

// Adapter for Position3D from Eigen Vector3f
template <>
struct CollectionAdapter<components::Position3D, std::vector<Eigen::Matrix<float, 3, 1>>> {
    Collection<components::Position3D> operator()(
        const std::vector<Eigen::Matrix<float, 3, 1>>& points) {
        std::vector<components::Position3D> converted;
        converted.reserve(points.size());
        for (const auto& p : points) {
            converted.push_back({p.x(), p.y(), p.z()});
        }
        return Collection<components::Position3D>(std::move(converted));
    }
};

// Adapter for Color from Eigen Vector3f
template <>
struct CollectionAdapter<components::Color, std::vector<Eigen::Matrix<float, 3, 1>>> {
    Collection<components::Color> operator()(
        const std::vector<Eigen::Matrix<float, 3, 1>>& colors) {
        std::vector<components::Color> converted;
        converted.reserve(colors.size());
        for (const auto& c : colors) {
            converted.push_back({c.x(), c.y(), c.z(), 1.0f});  // RGB with alpha=1
        }
        return Collection<components::Color>(std::move(converted));
    }
};

template <typename TElement>
struct rerun::CollectionAdapter<TElement, cv::Mat> {
    /// Borrow for non-temporary.
    Collection<TElement> operator()(const cv::Mat& img) {
        return Collection<TElement>::borrow(reinterpret_cast<TElement*>(img.data),
                                            img.total() * img.channels());
    }

    // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is
    // destroyed).
    Collection<TElement> operator()(cv::Mat&& img) {
        std::vector<TElement> img_vec(img.total() * img.channels());
        img_vec.assign(img.data, img.data + img.total() * img.channels());
        return Collection<TElement>::take_ownership(std::move(img_vec));
    }
};

}  // namespace rerun
