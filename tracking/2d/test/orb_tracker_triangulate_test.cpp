#include <2d/orb_tracker.hpp>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <logging/logging.hpp>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <stf/transform_utils.hpp>

// Helper function to project a 3D point to pixel coordinates
Eigen::Vector2d projectPointToPixel(const Eigen::Vector3d& point, const Eigen::Matrix3d& K) {
    Eigen::Vector3d pixel_homo = K * (point / point.z());
    Eigen::Vector2d pixel(pixel_homo.x(), pixel_homo.y());
    return pixel;
}

// Helper function to project a 3D point after applying a transform
Eigen::Vector2d projectPointWithTransform(const Eigen::Vector3d& point, const Eigen::Matrix3d& K,
                                          const Eigen::Isometry3d& T) {
    Eigen::Vector3d transformed_point = T.inverse() * point;
    assert(transformed_point.z() > 0);
    return projectPointToPixel(transformed_point, K);
}

// Helper function to create a KeyFrame with minimal required data
core::types::KeyFrame createTestKeyframe(uint64_t id, const std::string& frame_id,
                                         const Eigen::Vector3d& position,
                                         const Eigen::Quaterniond& orientation,
                                         const Eigen::Matrix3d& K) {
    core::types::KeyFrame kf;
    kf.id = id;
    kf.pose.position = position;
    kf.pose.orientation = orientation;
    kf.pose.frame_id = "odom";     // World frame
    kf.pose.timestamp = id * 0.1;  // Realistic timestamp spacing

    // Set up camera info
    core::types::CameraInfo cam_info;
    cam_info.frame_id = frame_id;
    cam_info.width = 1280;
    cam_info.height = 720;
    cam_info.distortion_model = "plumb_bob";

    // Convert Eigen matrix to row-major vector
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_rm = K;
    cam_info.k = std::vector<double>(K_rm.data(), K_rm.data() + K_rm.size());
    cam_info.d = std::vector<double>(5, 0.0);  // No distortion
    kf.camera_info = cam_info;

    // Create minimal image data
    core::types::Image img;
    img.frame_id = frame_id;
    img.width = 1280;
    img.height = 720;
    img.encoding = "bgr8";
    img.channels = 3;
    img.data = cv::Mat::zeros(720, 1280, CV_8UC3);  // Dummy image data
    kf.color_data = img;

    return kf;
}

//  Printing some debug info:
//
// 636.642       0  635.58
//       0 636.185 372.611
//       0       0       1
//
// 636.642       0  635.58
//       0 636.185 372.611
//       0       0       1
//  720x1280
//  720x1280
//  === PREVIOUS FRAME: base_link -> camera_color_optical_frame ===
//    Translation: 0.085     0   0.3
//    Rotation Matrix:
// 4.89653e-12 4.89653e-12           1
//          -1           0 4.89653e-12
//           0          -1 4.89653e-12
//    Quaternion (w,x,y,z): 0.5, -0.5, 0.5, -0.5
//    RPY (rad):   90 -180   90
//  === CURRENT FRAME: base_link -> camera_color_optical_frame ===
//    Translation: 0.085     0   0.3
//    Rotation Matrix:
// 4.89653e-12 4.89653e-12           1
//          -1           0 4.89653e-12
//           0          -1 4.89653e-12
//    Quaternion (w,x,y,z): 0.5, -0.5, 0.5, -0.5
//    RPY (rad):   90 -180   90

class OrbTrackerTriangulateTest : public ::testing::Test {
protected:
    Eigen::Matrix3d K;
    double tolerance;
    std::shared_ptr<tracking::image::OrbTracker> tracker;
    std::shared_ptr<stf::TransformTree> tft;

    OrbTrackerTriangulateTest() {
        // Realistic camera matrix (from actual rosbag data in comments)
        K << 636.642, 0, 635.58, 0, 636.185, 372.611, 0, 0, 1;
        tolerance = 0.05;  // 5cm tolerance for triangulation

        // Create tracker
        tracker = std::make_shared<tracking::image::OrbTracker>(500, 1.2f, 8);

        // Create transform tree with the exact transform from rosbag data
        // From comments: base_link -> camera_color_optical_frame
        // Translation: 0.085, 0, 0.3
        // Quaternion (w,x,y,z): 0.5, -0.5, 0.5, -0.5
        // RPY (rad): 90, -180, 90 (degrees in comments, but values are in radians)
        tft = std::make_shared<stf::TransformTree>();
        Eigen::Isometry3d T_base_camera = Eigen::Isometry3d::Identity();
        T_base_camera.translation() = Eigen::Vector3d(0.085, 0.0, 0.3);
        T_base_camera.linear() = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5).toRotationMatrix();
        LOG(INFO) << "T_base_camera rpy: " << stf::getRPY(T_base_camera).transpose();

        tft->setTransform("base_link", "camera_color_optical_frame", T_base_camera);
    }

    void SetUp() override {
        // Set up the transform tree in the tracker using public setters
        tracker->setTransformTree(tft);
        tracker->setBaseFrameId("base_link");
    }
};

// Test 1: Simple translation along X-axis (typical forward motion)
TEST_F(OrbTrackerTriangulateTest, SimpleForwardTranslation) {
    LOG(INFO) << "=== Test: Simple Forward Translation ===";

    // Setup: Robot moves 0.2m forward along X-axis
    Eigen::Vector3d pos_prev(0.0, 0.0, 0.0);
    Eigen::Vector3d pos_cur(0.2, 0.0, 0.0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    // Create keyframes
    core::types::KeyFrame prev_kf =
        createTestKeyframe(0, "camera_color_optical_frame", pos_prev, orientation, K);
    core::types::KeyFrame cur_kf =
        createTestKeyframe(1, "camera_color_optical_frame", pos_cur, orientation, K);

    // Define a known 3D point in world frame (in front of both camera positions)
    Eigen::Vector3d world_point(1.0, 0.3, 0.5);  // 1m forward, 0.3m right, 0.5m up

    // Transform point to both camera frames for projection
    Eigen::Isometry3d T_world_prev = prev_kf.pose.getEigenIsometry();
    Eigen::Isometry3d T_world_cur = cur_kf.pose.getEigenIsometry();

    Eigen::Vector3d point_prev_bl = T_world_prev.inverse() * world_point;
    Eigen::Vector3d point_cur_bl = T_world_cur.inverse() * world_point;

    auto T_base_camera = tft->getTransform("base_link", "camera_color_optical_frame").transform;
    Eigen::Vector3d point_prev_cam = T_base_camera.inverse() * point_prev_bl;
    Eigen::Vector3d point_cur_cam = T_base_camera.inverse() * point_cur_bl;

    LOG(INFO) << "World point: " << world_point.transpose();
    LOG(INFO) << "Prev camera point: " << point_prev_bl.transpose();
    LOG(INFO) << "Cur camera point: " << point_cur_bl.transpose();
    LOG(INFO) << "Prev camera point (base_link): " << point_prev_cam.transpose();
    LOG(INFO) << "Cur camera point (base_link): " << point_cur_cam.transpose();

    ASSERT_GT(point_prev_cam.z(), 0) << "Point must be in front of previous camera";
    ASSERT_GT(point_cur_cam.z(), 0) << "Point must be in front of current camera";

    // Project to pixel coordinates
    Eigen::Vector2d pixel_prev = projectPointToPixel(point_prev_cam, K);
    Eigen::Vector2d pixel_cur = projectPointToPixel(point_cur_cam, K);

    LOG(INFO) << "World point: " << world_point.transpose();
    LOG(INFO) << "Prev camera point: " << point_prev_cam.transpose();
    LOG(INFO) << "Cur camera point: " << point_cur_cam.transpose();
    LOG(INFO) << "Prev pixel: " << pixel_prev.transpose();
    LOG(INFO) << "Cur pixel: " << pixel_cur.transpose();

    // Create OpenCV point vectors
    std::vector<cv::Point2f> prev_points = {cv::Point2f(pixel_prev.x(), pixel_prev.y())};
    std::vector<cv::Point2f> cur_points = {cv::Point2f(pixel_cur.x(), pixel_cur.y())};

    // Convert camera matrix to cv::Mat
    cv::Mat K_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_cv.at<double>(i, j) = K(i, j);
        }
    }
    LOG(INFO) << "K:\n" << K_cv;

    // Call triangulateMatches
    std::vector<Eigen::Vector3d> result =
        tracker->triangulateMatches(prev_points, cur_points, prev_kf, cur_kf, K_cv);

    // Verify results
    ASSERT_EQ(result.size(), 1) << "Should triangulate exactly one point";

    LOG(INFO) << "Triangulated point: " << result[0].transpose();
    LOG(INFO) << "Error: " << (result[0] - world_point).norm() << "m";

    EXPECT_NEAR(result[0].x(), world_point.x(), tolerance) << "X coordinate mismatch";
    EXPECT_NEAR(result[0].y(), world_point.y(), tolerance) << "Y coordinate mismatch";
    EXPECT_NEAR(result[0].z(), world_point.z(), tolerance) << "Z coordinate mismatch";
}

// Test 2: Translation with rotation (realistic robot motion)
TEST_F(OrbTrackerTriangulateTest, TranslationWithRotation) {
    LOG(INFO) << "=== Test: Translation with Rotation ===";

    // Setup: Robot moves 0.15m forward and rotates 5 degrees around Z (yaw)
    Eigen::Vector3d pos_prev(0.0, 0.0, 0.0);
    Eigen::Vector3d pos_cur(0.15, 0.0, 0.0);

    Eigen::Quaterniond orient_prev = Eigen::Quaterniond::Identity();
    Eigen::AngleAxisd rotation_cur(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond orient_cur(rotation_cur);

    // Create keyframes (using the rosbag transform set in constructor)
    core::types::KeyFrame prev_kf =
        createTestKeyframe(0, "camera_color_optical_frame", pos_prev, orient_prev, K);
    core::types::KeyFrame cur_kf =
        createTestKeyframe(1, "camera_color_optical_frame", pos_cur, orient_cur, K);

    // Multiple 3D points in world frame
    std::vector<Eigen::Vector3d> world_points = {
        Eigen::Vector3d(1.2, 0.3, 0.5),   // Forward-right
        Eigen::Vector3d(0.8, -0.2, 0.3),  // Forward-left
        Eigen::Vector3d(1.5, 0.0, 0.8)    // Far-center
    };

    std::vector<cv::Point2f> prev_points;
    std::vector<cv::Point2f> cur_points;

    // Get camera poses in world frame (using transform from TFT)
    auto T_base_prev = prev_kf.pose.getEigenIsometry();
    auto T_base_cur = cur_kf.pose.getEigenIsometry();
    auto T_base_camera = tft->getTransform("base_link", "camera_color_optical_frame").transform;
    auto T_world_camera_prev = T_base_prev * T_base_camera;
    auto T_world_camera_cur = T_base_cur * T_base_camera;

    for (const auto& world_point : world_points) {
        Eigen::Vector3d point_prev_cam = T_world_camera_prev.inverse() * world_point;
        Eigen::Vector3d point_cur_cam = T_world_camera_cur.inverse() * world_point;

        if (point_prev_cam.z() > 0 && point_cur_cam.z() > 0) {
            Eigen::Vector2d pixel_prev = projectPointToPixel(point_prev_cam, K);
            Eigen::Vector2d pixel_cur = projectPointToPixel(point_cur_cam, K);

            prev_points.push_back(cv::Point2f(pixel_prev.x(), pixel_prev.y()));
            cur_points.push_back(cv::Point2f(pixel_cur.x(), pixel_cur.y()));
        }
    }

    ASSERT_EQ(prev_points.size(), world_points.size()) << "All points should be visible";

    cv::Mat K_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_cv.at<double>(i, j) = K(i, j);
        }
    }

    // Call triangulateMatches
    std::vector<Eigen::Vector3d> result =
        tracker->triangulateMatches(prev_points, cur_points, prev_kf, cur_kf, K_cv);

    // Verify results
    ASSERT_EQ(result.size(), world_points.size())
        << "Should triangulate all " << world_points.size() << " points";

    for (size_t i = 0; i < result.size(); ++i) {
        double error = (result[i] - world_points[i]).norm();
        LOG(INFO) << "Point " << i << " - Expected: " << world_points[i].transpose()
                  << ", Got: " << result[i].transpose() << ", Error: " << error << "m";

        EXPECT_LT(error, tolerance) << "Point " << i << " reconstruction error too large";
    }
}

// Test 3: Realistic rosbag-like motion (typical indoor robot movement)
TEST_F(OrbTrackerTriangulateTest, RealisticRosbagMotion) {
    LOG(INFO) << "=== Test: Realistic Rosbag Motion ===";

    // Realistic scenario: Robot moves ~10cm forward with slight rotation over 0.1 seconds
    // This simulates typical keyframe spacing in a rosbag at ~10Hz with ~1m/s velocity
    Eigen::Vector3d pos_prev(2.5, 1.3, 0.0);  // Arbitrary world position
    Eigen::Vector3d pos_cur(2.6, 1.32, 0.0);  // Moved ~10cm forward, slight lateral

    // Small rotation: 3 degrees yaw
    Eigen::AngleAxisd rot_prev(12.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rot_cur(15.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond orient_prev(rot_prev);
    Eigen::Quaterniond orient_cur(rot_cur);

    // Use realistic camera matrix from actual rosbag data
    Eigen::Matrix3d K_real;
    K_real << 636.642, 0, 635.58, 0, 636.185, 372.611, 0, 0, 1;

    core::types::KeyFrame prev_kf =
        createTestKeyframe(71, "camera_color_optical_frame", pos_prev, orient_prev, K_real);
    core::types::KeyFrame cur_kf =
        createTestKeyframe(72, "camera_color_optical_frame", pos_cur, orient_cur, K_real);

    // Define points at typical indoor distances (0.5m - 5m)
    std::vector<Eigen::Vector3d> world_points = {
        Eigen::Vector3d(3.0, 1.5, 0.5),  // Wall feature ~0.5m away
        Eigen::Vector3d(4.2, 1.8, 0.3),  // Floor feature ~2m away
        Eigen::Vector3d(5.5, 0.9, 1.2),  // Ceiling feature ~3m away
        Eigen::Vector3d(3.5, 2.1, 0.7)   // Object ~1m away
    };

    std::vector<cv::Point2f> prev_points;
    std::vector<cv::Point2f> cur_points;

    auto T_base_prev = prev_kf.pose.getEigenIsometry();
    auto T_base_cur = cur_kf.pose.getEigenIsometry();
    auto T_base_camera = tft->getTransform("base_link", "camera_color_optical_frame").transform;
    auto T_world_camera_prev = T_base_prev * T_base_camera;
    auto T_world_camera_cur = T_base_cur * T_base_camera;

    LOG(INFO) << "Previous camera pose: " << pos_prev.transpose()
              << ", rotation: " << stf::getRPY(T_world_camera_prev).transpose();
    LOG(INFO) << "Current camera pose: " << pos_cur.transpose()
              << ", rotation: " << stf::getRPY(T_world_camera_cur).transpose();

    for (const auto& world_point : world_points) {
        Eigen::Vector3d point_prev_cam = T_world_camera_prev.inverse() * world_point;
        Eigen::Vector3d point_cur_cam = T_world_camera_cur.inverse() * world_point;

        if (point_prev_cam.z() > 0 && point_cur_cam.z() > 0) {
            Eigen::Vector2d pixel_prev = projectPointToPixel(point_prev_cam, K_real);
            Eigen::Vector2d pixel_cur = projectPointToPixel(point_cur_cam, K_real);

            // Check if within image bounds (1280x720 typical resolution)
            if (pixel_prev.x() >= 0 && pixel_prev.x() < 1280 && pixel_prev.y() >= 0 &&
                pixel_prev.y() < 720 && pixel_cur.x() >= 0 && pixel_cur.x() < 1280 &&
                pixel_cur.y() >= 0 && pixel_cur.y() < 720) {
                prev_points.push_back(cv::Point2f(pixel_prev.x(), pixel_prev.y()));
                cur_points.push_back(cv::Point2f(pixel_cur.x(), pixel_cur.y()));
            }
        }
    }

    ASSERT_GT(prev_points.size(), 0) << "At least some points should be visible";

    cv::Mat K_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_cv.at<double>(i, j) = K_real(i, j);
        }
    }

    // Call triangulateMatches
    std::vector<Eigen::Vector3d> result =
        tracker->triangulateMatches(prev_points, cur_points, prev_kf, cur_kf, K_cv);

    LOG(INFO) << "Triangulated " << result.size() << " out of " << prev_points.size()
              << " point correspondences";

    // Verify that we get reasonable results (some points may be filtered out)
    ASSERT_GT(result.size(), 0) << "Should triangulate at least some points";
    ASSERT_LE(result.size(), prev_points.size()) << "Cannot triangulate more points than input";

    // Check that triangulated points are reasonable
    for (size_t i = 0; i < result.size(); ++i) {
        LOG(INFO) << "Triangulated point " << i << ": " << result[i].transpose();

        // Check basic sanity: points should be in reasonable range
        EXPECT_GT(result[i].z(), -1.0) << "Point " << i << " z-coordinate unreasonable";
        EXPECT_LT(result[i].z(), 10.0) << "Point " << i << " z-coordinate unreasonable";

        double distance_from_robot = (result[i] - pos_cur).norm();
        EXPECT_LT(distance_from_robot, 100.0)
            << "Point " << i << " too far from robot (likely triangulation failure)";
    }
}

// Test 4: Edge case - very small baseline (should still work with good geometry)
TEST_F(OrbTrackerTriangulateTest, SmallBaseline) {
    LOG(INFO) << "=== Test: Small Baseline ===";

    // Very small motion: 2cm translation
    Eigen::Vector3d pos_prev(0.0, 0.0, 0.0);
    Eigen::Vector3d pos_cur(0.02, 0.0, 0.0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    core::types::KeyFrame prev_kf =
        createTestKeyframe(0, "camera_color_optical_frame", pos_prev, orientation, K);
    core::types::KeyFrame cur_kf =
        createTestKeyframe(1, "camera_color_optical_frame", pos_cur, orientation, K);

    // Close point (small baseline requires close points for good triangulation)
    Eigen::Vector3d world_point(0.5, 0.1, 0.2);

    auto T_world_prev = prev_kf.pose.getEigenIsometry();
    auto T_world_cur = cur_kf.pose.getEigenIsometry();

    Eigen::Vector3d point_prev_cam = T_world_prev.inverse() * world_point;
    Eigen::Vector3d point_cur_cam = T_world_cur.inverse() * world_point;

    Eigen::Vector2d pixel_prev = projectPointToPixel(point_prev_cam, K);
    Eigen::Vector2d pixel_cur = projectPointToPixel(point_cur_cam, K);

    std::vector<cv::Point2f> prev_points = {cv::Point2f(pixel_prev.x(), pixel_prev.y())};
    std::vector<cv::Point2f> cur_points = {cv::Point2f(pixel_cur.x(), pixel_cur.y())};

    cv::Mat K_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_cv.at<double>(i, j) = K(i, j);
        }
    }

    std::vector<Eigen::Vector3d> result =
        tracker->triangulateMatches(prev_points, cur_points, prev_kf, cur_kf, K_cv);

    // With small baseline, triangulation may be less accurate but should still work
    if (result.size() > 0) {
        double error = (result[0] - world_point).norm();
        LOG(INFO) << "Small baseline error: " << error << "m";
        // Allow larger tolerance for small baseline
        EXPECT_LT(error, 0.2) << "Even with small baseline, error should be reasonable";
    } else {
        LOG(WARNING) << "Small baseline case filtered out by quality checks (acceptable)";
    }
}

// Test 5: Multiple points at varying depths
TEST_F(OrbTrackerTriangulateTest, VaryingDepths) {
    LOG(INFO) << "=== Test: Varying Depths ===";

    Eigen::Vector3d pos_prev(0.0, 0.0, 0.0);
    Eigen::Vector3d pos_cur(0.2, 0.0, 0.0);
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

    core::types::KeyFrame prev_kf =
        createTestKeyframe(0, "camera_color_optical_frame", pos_prev, orientation, K);
    core::types::KeyFrame cur_kf =
        createTestKeyframe(1, "camera_color_optical_frame", pos_cur, orientation, K);

    // Points at different depths: close, medium, far
    std::vector<Eigen::Vector3d> world_points = {
        Eigen::Vector3d(0.5, 0.0, 0.2),  // Close: 0.5m
        Eigen::Vector3d(2.0, 0.3, 0.5),  // Medium: 2m
        Eigen::Vector3d(5.0, -0.2, 0.8)  // Far: 5m
    };

    std::vector<cv::Point2f> prev_points;
    std::vector<cv::Point2f> cur_points;

    auto T_world_prev = prev_kf.pose.getEigenIsometry();
    auto T_world_cur = cur_kf.pose.getEigenIsometry();

    for (const auto& world_point : world_points) {
        Eigen::Vector3d point_prev_cam = T_world_prev.inverse() * world_point;
        Eigen::Vector3d point_cur_cam = T_world_cur.inverse() * world_point;

        Eigen::Vector2d pixel_prev = projectPointToPixel(point_prev_cam, K);
        Eigen::Vector2d pixel_cur = projectPointToPixel(point_cur_cam, K);

        prev_points.push_back(cv::Point2f(pixel_prev.x(), pixel_prev.y()));
        cur_points.push_back(cv::Point2f(pixel_cur.x(), pixel_cur.y()));
    }

    cv::Mat K_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_cv.at<double>(i, j) = K(i, j);
        }
    }

    std::vector<Eigen::Vector3d> result =
        tracker->triangulateMatches(prev_points, cur_points, prev_kf, cur_kf, K_cv);

    ASSERT_EQ(result.size(), world_points.size()) << "Should triangulate all depth variations";

    for (size_t i = 0; i < result.size(); ++i) {
        double error = (result[i] - world_points[i]).norm();
        double depth = world_points[i].x();  // Forward distance
        LOG(INFO) << "Depth " << depth << "m - Error: " << error << "m";

        // Farther points may have slightly higher error
        double depth_scaled_tolerance = tolerance * (1.0 + depth / 10.0);
        EXPECT_LT(error, depth_scaled_tolerance)
            << "Point at depth " << depth << "m has excessive error";
    }
}
