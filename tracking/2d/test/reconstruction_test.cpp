#include <2d/reconstruction.hpp>
#include <cassert>
#include <filesystem>
#include <logging/logging.hpp>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <stf/transform_utils.hpp>

Eigen::Vector2d projectKnownPointToPixel(const Eigen::Vector3d& point, const Eigen::Matrix3d& K) {
    Eigen::Vector3d pixel_homo = K * (point / point.z());
    Eigen::Vector2d pixel(pixel_homo.x(), pixel_homo.y());
    return pixel;
}

Eigen::Vector2d projectKnownPointToTransformedCameraPixel(const Eigen::Vector3d& point,
                                                          const Eigen::Matrix3d& K,
                                                          const Eigen::Isometry3d& T_c2_c1) {
    Eigen::Vector3d point_c2 = T_c2_c1.inverse() * point;
    assert(point_c2.z() > 0);
    return projectKnownPointToPixel(point_c2, K);
}

core::types::KeyFrame createKeyframe(uint32_t i, const std::string& frame_id,
                                     const Eigen::Vector3d& pos, const Eigen::Quaterniond& q,
                                     const Eigen::Matrix3d& K) {
    core::types::KeyFrame kf;
    kf.id = i;
    kf.pose.position = pos;
    kf.pose.orientation = q;
    core::types::CameraInfo c;
    c.frame_id = frame_id;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_rm = K;
    c.k = std::vector<double>(K_rm.data(), K_rm.data() + K_rm.size());
    kf.camera_info = c;
    core::types::Image img;
    img.frame_id = frame_id;
    kf.color_data = img;
    return kf;
}

// Helper function to create a KeyFrame with real image data from /data/robot/keyframes
core::types::KeyFrame createKeyframeWithRealImage(uint32_t keyframe_id, const std::string& frame_id,
                                                  const Eigen::Vector3d& pos,
                                                  const Eigen::Quaterniond& q,
                                                  const Eigen::Matrix3d& K) {
    // Try to load the real image
    std::string image_path =
        "/data/robot/keyframes/keyframe_" + std::to_string(keyframe_id) + ".png";

    // Check if the image file exists (same as map_store_test)
    if (!std::filesystem::exists(image_path)) {
        // Return empty keyframe if image doesn't exist - caller should handle this
        core::types::KeyFrame empty_kf;
        empty_kf.id = keyframe_id;
        return empty_kf;
    }

    // Load the actual keyframe image (same as map_store_test)
    cv::Mat real_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (real_image.empty()) {
        // Return empty keyframe if loading fails - caller should handle this
        core::types::KeyFrame empty_kf;
        empty_kf.id = keyframe_id;
        return empty_kf;
    }

    // Create keyframe with real image data (same structure as map_store_test)
    core::types::KeyFrame kf;
    kf.id = keyframe_id;
    kf.pose.position = pos;
    kf.pose.orientation = q;
    kf.pose.frame_id = frame_id;

    // Set up camera info to match the real image dimensions (same as map_store_test)
    core::types::CameraInfo c;
    c.frame_id = frame_id;
    c.width = real_image.cols;
    c.height = real_image.rows;
    c.distortion_model = "plumb_bob";

    // Use provided camera matrix (same as map_store_test)
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_rm = K;
    c.k = std::vector<double>(K_rm.data(), K_rm.data() + K_rm.size());
    c.d = std::vector<double>(5, 0.0);  // No distortion (same as map_store_test)
    kf.camera_info = c;

    // Create image from the loaded cv::Mat (same as map_store_test)
    core::types::Image img = core::types::Image::fromCvMat(real_image, "bgr8", frame_id);
    kf.color_data = img;

    return kf;
}

class TriangulationTest : public ::testing::Test {
protected:
    Eigen::Matrix3d K;
    double tolerance;
    tracking::image::Reconstruct reconstruct;

    TriangulationTest() {
        K << 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0;
        tolerance = 1e-7;
    }
};

TEST_F(TriangulationTest, KnownPointSimpleTranslation) {
    Eigen::Vector3d X_known_cam1(0.1, 0.05, 2.0);  // Point in cam1 frame
    Eigen::Isometry3d T_c2_c1 = Eigen::Isometry3d::Identity();
    T_c2_c1.translation() = Eigen::Vector3d(0.2, 0, 0);  // Cam2 moved 0.2m along X-axis of Cam1

    Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q1(p1.rotation());

    stf::TransformTree tft;
    tft.setTransform("base_link", "camera1", Eigen::Isometry3d::Identity());

    core::types::KeyFrame kf1 = createKeyframe(0, "camera1", p1.translation(), q1, K);
    core::types::KeyFrame kf2 = createKeyframe(0, "camera1", T_c2_c1.translation(), q1, K);

    auto p1_px = projectKnownPointToPixel(X_known_cam1, K);
    auto p2_px = projectKnownPointToTransformedCameraPixel(X_known_cam1, K, T_c2_c1);
    ASSERT_FALSE(std::isnan(p1_px.x()) || std::isnan(p2_px.x()));

    std::vector<cv::Point2f> points1_vec = {cv::Point2f(p1_px.x(), p1_px.y())};
    std::vector<cv::Point2f> points2_vec = {cv::Point2f(p2_px.x(), p2_px.y())};

    std::vector<Eigen::Vector3d> result;
    Eigen::Isometry3d essentialMat = Eigen::Isometry3d::Identity();
    ASSERT_TRUE(reconstruct.triangulate(kf1, kf2, points1_vec, points2_vec, tft, essentialMat,
                                        result, false));

    ASSERT_EQ(result.size(), 1);
    ASSERT_FALSE(std::isnan(result[0].x()));
    EXPECT_NEAR(result[0].x(), X_known_cam1.x(), tolerance);
    EXPECT_NEAR(result[0].y(), X_known_cam1.y(), tolerance);
    EXPECT_NEAR(result[0].z(), X_known_cam1.z(), tolerance);
}

TEST_F(TriangulationTest, TriangulationWithRealImages) {
    std::string image_path_71 = "/data/robot/keyframes/keyframe_71.png";
    std::string image_path_72 = "/data/robot/keyframes/keyframe_72.png";

    if (!std::filesystem::exists(image_path_71) || !std::filesystem::exists(image_path_72)) {
        GTEST_SKIP()
            << "Required keyframe images not found. Need keyframe_71.png and keyframe_72.png";
        return;
    }

    Eigen::Matrix3d K_real;
    K_real << 636.642, 0, 635.58, 0, 636.185, 372.611, 0, 0, 1;

    // Use stereo camera configuration with realistic rotations for testing
    Eigen::Vector3d pos1(0.0, 0.0, 0.0);  // Camera 1 at origin
    // Add slight rotation to camera 1 (5 degrees around Y-axis)
    Eigen::AngleAxisd rotation1(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q1(rotation1);

    Eigen::Vector3d pos2(0.1, 0.0, 0.0);  // Camera 2 translated 10cm along X-axis
    // Add slight rotation to camera 2 (-3 degrees around Y-axis, 2 degrees around X-axis)
    Eigen::AngleAxisd rotationY(-3.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotationX(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q2 = rotationY * rotationX;

    // Calculate camera poses in world frame
    Eigen::Isometry3d T_world_cam1 = Eigen::Isometry3d::Identity();
    T_world_cam1.translation() = pos1;
    T_world_cam1.linear() = q1.toRotationMatrix();

    Eigen::Isometry3d T_world_cam2 = Eigen::Isometry3d::Identity();
    T_world_cam2.translation() = pos2;
    T_world_cam2.linear() = q2.toRotationMatrix();

    core::types::KeyFrame kf1 = createKeyframeWithRealImage(71, "camera1", pos1, q1, K_real);
    core::types::KeyFrame kf2 = createKeyframeWithRealImage(72, "camera1", pos2, q2, K_real);

    ASSERT_TRUE(kf1.color_data.has_value());
    ASSERT_TRUE(kf2.color_data.has_value());
    ASSERT_FALSE(kf1.color_data.value().data.empty());
    ASSERT_FALSE(kf2.color_data.value().data.empty());

    stf::TransformTree tft;
    tft.setTransform("base_link", "camera1", Eigen::Isometry3d::Identity());

    // Define realistic 3D points in WORLD coordinates that would be visible from both camera
    // positions These points are in the world coordinate frame, not camera1 frame
    std::vector<Eigen::Vector3d> world_points = {
        Eigen::Vector3d(0.5, 0.3, 1.0),   // Point in world frame
        Eigen::Vector3d(-0.3, 0.2, 1.2),  // Point in world frame
        Eigen::Vector3d(0.1, -0.4, 0.8)   // Point in world frame
    };

    // Project the 3D world points to both cameras using their world poses
    std::vector<cv::Point2f> points1_vec;
    std::vector<cv::Point2f> points2_vec;

    for (const auto& world_point : world_points) {
        // Transform world point to camera1 frame
        Eigen::Vector3d point_cam1 = T_world_cam1.inverse() * world_point;

        // Transform world point to camera2 frame
        Eigen::Vector3d point_cam2 = T_world_cam2.inverse() * world_point;

        // Check if points are in front of both cameras
        if (point_cam1.z() <= 0 || point_cam2.z() <= 0) {
            continue;  // Skip points behind cameras
        }

        // Project to image coordinates
        Eigen::Vector2d pixel1 = projectKnownPointToPixel(point_cam1, K_real);
        Eigen::Vector2d pixel2 = projectKnownPointToPixel(point_cam2, K_real);

        // Check if projections are valid (within image bounds)
        if (!std::isnan(pixel1.x()) && !std::isnan(pixel1.y()) && !std::isnan(pixel2.x()) &&
            !std::isnan(pixel2.y()) && pixel1.x() >= 0 &&
            pixel1.x() < kf1.color_data.value().width && pixel1.y() >= 0 &&
            pixel1.y() < kf1.color_data.value().height && pixel2.x() >= 0 &&
            pixel2.x() < kf2.color_data.value().width && pixel2.y() >= 0 &&
            pixel2.y() < kf2.color_data.value().height) {
            points1_vec.push_back(cv::Point2f(pixel1.x(), pixel1.y()));
            points2_vec.push_back(cv::Point2f(pixel2.x(), pixel2.y()));

            LOG(INFO) << "Valid point projection: world=" << world_point.transpose()
                      << " -> cam1_pix=[" << pixel1.transpose() << "] cam2_pix=["
                      << pixel2.transpose() << "]";
        }
    }

    // Ensure we have at least one valid point correspondence
    ASSERT_GT(points1_vec.size(), 0) << "No valid point correspondences found";
    EXPECT_EQ(points1_vec.size(), points2_vec.size());

    std::vector<Eigen::Vector3d> result;
    Eigen::Isometry3d essentialMat = stf::getRelative(kf1, kf2, tft);
    LOG(INFO) << "Using essential matrix:\n"
              << essentialMat.translation().transpose() << " "
              << stf::getRPY(essentialMat).transpose();

    bool success = reconstruct.triangulate(kf1, kf2, points1_vec, points2_vec, tft, essentialMat,
                                           result, false);
    ASSERT_TRUE(success);

    EXPECT_EQ(result.size(), points1_vec.size());

    // Verify triangulated points are close to original world points
    for (size_t i = 0; i < result.size() && i < world_points.size(); ++i) {
        EXPECT_FALSE(std::isnan(result[i].x()));
        EXPECT_FALSE(std::isnan(result[i].y()));
        EXPECT_FALSE(std::isnan(result[i].z()));

        // Check if triangulated point is reasonably close to original
        // (allowing for some error due to discretization and numerical precision)
        double distance_error = (result[i] - world_points[i]).norm();
        EXPECT_LT(distance_error, 0.5) << "Triangulated point " << i << " too far from original. "
                                       << "Original: (" << world_points[i].transpose() << "), "
                                       << "Triangulated: (" << result[i].transpose() << "), "
                                       << "Error: " << distance_error;
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
