#include <gtest/gtest.h>
#include "tracking/proto_utils.hpp"
#include "tracking/2d/reconstruction.hpp"
#include "tracking/2d/orb_tracker.hpp"
#include "core/types/keyframe.hpp"
#include "stf/transform_tree.hpp"

// Define a test fixture if needed, or use TEST directly
class TrackingPipelineTest : public ::testing::Test {
protected:
    tracking::proto::TrackingTestCase test_case_;
    std::string test_data_path_ = "tracking/test_data/pipeline_case_1.pb"; // Path from bazel runfiles

    void SetUp() override {
        // Locate the test data file. Bazel places data dependencies in a specific runfiles path.
        // For simplicity, assuming the path is relative to the workspace root when running the test.
        // If running via bazel test, you might need to use runfiles library or environment variables to locate data.
        // std::string runfiles_manifest_file;
        // const char* manifest_env = std::getenv("RUNFILES_MANIFEST_FILE");
        // if (manifest_env) { runfiles_manifest_file = manifest_env; }
        // If runfiles are used, actual path might be complex.
        // For now, assume it's accessible via relative path from execution dir.

        test_case_ = tracking::proto_utils::loadTrackingTestCase(test_data_path_);
        ASSERT_TRUE(test_case_.has_keyframe1()); // Basic check that data loaded
    }
};

TEST_F(TrackingPipelineTest, TriangulatePointsFromProto) {
    ASSERT_TRUE(test_case_.has_keyframe1());
    ASSERT_TRUE(test_case_.has_keyframe2());
    ASSERT_TRUE(test_case_.has_transform_tree());
    ASSERT_TRUE(test_case_.has_matches());

    core::types::KeyFrame kf1 = tracking::proto_utils::fromProto(test_case_.keyframe1());
    core::types::KeyFrame kf2 = tracking::proto_utils::fromProto(test_case_.keyframe2());
    stf::TransformTree tft = tracking::proto_utils::fromProto(test_case_.transform_tree());

    std::vector<cv::Point2f> points1_cv, points2_cv;
    tracking::proto_utils::fromProto(test_case_.matches(), points1_cv, points2_cv);

    ASSERT_FALSE(points1_cv.empty());
    ASSERT_EQ(points1_cv.size(), points2_cv.size());

    tracking::image::Reconstruct reconstructor;
    std::vector<Eigen::Vector3d> triangulated_points_world;

    // Assuming use_essential_mat is false for this pipeline test using TF tree for relative pose
    // and essentialMatrix argument is ignored if use_essential_mat is false.
    // Pass a dummy essential matrix if the API requires it even when not used.
    Eigen::Isometry3d dummy_essential_matrix = Eigen::Isometry3d::Identity();

    bool success = reconstructor.triangulate(kf1, kf2, points1_cv, points2_cv, tft,
                                             dummy_essential_matrix, triangulated_points_world, false);

    ASSERT_TRUE(success);
    ASSERT_FALSE(triangulated_points_world.empty());
    ASSERT_EQ(triangulated_points_world.size(), test_case_.ground_truth_points_world_size());

    double tolerance = 1e-5;
    for (int i = 0; i < triangulated_points_world.size(); ++i) {
        const auto& p_world_proto = test_case_.ground_truth_points_world(i);
        EXPECT_NEAR(triangulated_points_world[i].x(), p_world_proto.x(), tolerance);
        EXPECT_NEAR(triangulated_points_world[i].y(), p_world_proto.y(), tolerance);
        EXPECT_NEAR(triangulated_points_world[i].z(), p_world_proto.z(), tolerance);
        std::cout << "Triangulated: " << triangulated_points_world[i].transpose()
                  << " vs GT: " << p_world_proto.x() << ", " << p_world_proto.y() << ", " << p_world_proto.z() << std::endl;
    }
}

// Add more tests for OrbTracker + Reconstruction if image data is included.