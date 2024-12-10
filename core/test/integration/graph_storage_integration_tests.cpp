// core/test/integration/graph_storage_integration_test.cpp
#include <gtest/gtest.h>
#include <core/graph/factor_graph.hpp>
#include <core/storage/map_store.hpp>

class GraphStorageIntegrationTest : public ::testing::Test {
protected:
    core::graph::FactorGraph graph_;
    core::storage::MapStore store_;

    // Helper functions
    core::types::KeyFrame::Ptr createKeyFrame(uint64_t id, double x, double y, double z) {
        auto kf = std::make_shared<core::types::KeyFrame>();
        kf->id = id;
        kf->pose.position = Eigen::Vector3d(x, y, z);
        kf->pose.orientation = Eigen::Quaterniond::Identity();
        kf->pose.timestamp = id * 1.0;

        // Add some test point cloud data
        core::types::PointCloud cloud;
        cloud.points.push_back(Eigen::Vector3d(x, y, z));
        cloud.colors.push_back(Eigen::Vector3d(1, 0, 0));
        kf->data = cloud;

        return kf;
    }

    core::types::Factor createOdometryFactor(uint64_t id, uint64_t from_id, uint64_t to_id,
                                           const Eigen::Vector3d& relative_translation) {
        core::types::Factor factor;
        factor.id = id;
        factor.type = core::proto::FactorType::ODOMETRY;
        factor.connected_nodes = {from_id, to_id};

        core::types::Pose relative_pose;
        relative_pose.position = relative_translation;
        relative_pose.orientation = Eigen::Quaterniond::Identity();
        factor.measurement.emplace<0>(relative_pose);

        factor.information = Eigen::Matrix<double, 6, 6>::Identity();
        return factor;
    }
};

TEST_F(GraphStorageIntegrationTest, OptimizeAndStore) {
    // Create a simple scenario with three keyframes in a triangle
    auto kf1 = createKeyFrame(1, 0, 0, 0);
    auto kf2 = createKeyFrame(2, 1, 0, 0);
    auto kf3 = createKeyFrame(3, 1, 1, 0);

    // Add keyframes to both graph and storage
    graph_.addKeyFrame(kf1);
    graph_.addKeyFrame(kf2);
    graph_.addKeyFrame(kf3);

    store_.addKeyFrame(kf1);
    store_.addKeyFrame(kf2);
    store_.addKeyFrame(kf3);

    // Create factors
    auto prior = core::types::Factor();
    prior.id = 1;
    prior.type = core::proto::FactorType::PRIOR;
    prior.connected_nodes = {1};
    prior.measurement.emplace<0>(kf1->pose);
    prior.information = Eigen::Matrix<double, 6, 6>::Identity();

    auto odom1 = createOdometryFactor(2, 1, 2, Eigen::Vector3d(1, 0, 0));
    auto odom2 = createOdometryFactor(3, 2, 3, Eigen::Vector3d(0, 1, 0));

    // Add factors to both graph and storage
    graph_.addFactor(prior);
    graph_.addFactor(odom1);
    graph_.addFactor(odom2);

    store_.addFactor(prior);
    store_.addFactor(odom1);
    store_.addFactor(odom2);

    // Optimize graph
    EXPECT_TRUE(graph_.optimize());

    // Get optimized poses and update storage
    auto optimized_keyframes = graph_.getAllKeyFrames();
    for (const auto& kf : optimized_keyframes) {
        store_.addKeyFrame(kf);  // This should update existing keyframes
    }

    // Save and reload the map
    const std::string test_file = "/tmp/test_optimized_map.pb";
    EXPECT_TRUE(store_.save(test_file));

    core::storage::MapStore loaded_store;
    EXPECT_TRUE(loaded_store.load(test_file));

    // Verify loaded data
    auto loaded_kf1 = loaded_store.getKeyFrame(1);
    auto loaded_kf2 = loaded_store.getKeyFrame(2);
    auto loaded_kf3 = loaded_store.getKeyFrame(3);

    ASSERT_NE(loaded_kf1, nullptr);
    ASSERT_NE(loaded_kf2, nullptr);
    ASSERT_NE(loaded_kf3, nullptr);

    // Check if optimized poses were properly saved and loaded
    EXPECT_TRUE(loaded_kf1->pose.position.isApprox(Eigen::Vector3d(0, 0, 0), 1e-5));
    EXPECT_TRUE(loaded_kf2->pose.position.isApprox(Eigen::Vector3d(1, 0, 0), 1e-5));
    EXPECT_TRUE(loaded_kf3->pose.position.isApprox(Eigen::Vector3d(1, 1, 0), 1e-5));
}

TEST_F(GraphStorageIntegrationTest, IncrementalUpdateAndStore) {
    // Initial keyframes
    auto kf1 = createKeyFrame(1, 0, 0, 0);
    auto kf2 = createKeyFrame(2, 1, 0, 0);

    // Add initial keyframes and factors
    graph_.addKeyFrame(kf1);
    graph_.addKeyFrame(kf2);
    store_.addKeyFrame(kf1);
    store_.addKeyFrame(kf2);

    auto prior = core::types::Factor();
    prior.id = 1;
    prior.type = core::proto::FactorType::PRIOR;
    prior.connected_nodes = {1};
    prior.measurement.emplace<0>(kf1->pose);
    prior.information = Eigen::Matrix<double, 6, 6>::Identity();

    auto odom1 = createOdometryFactor(2, 1, 2, Eigen::Vector3d(1, 0, 0));

    graph_.addFactor(prior);
    graph_.addFactor(odom1);
    store_.addFactor(prior);
    store_.addFactor(odom1);

    // First optimization
    EXPECT_TRUE(graph_.optimize());

    // Add new keyframe and factor
    auto kf3 = createKeyFrame(3, 2, 0, 0);
    graph_.addKeyFrame(kf3);
    store_.addKeyFrame(kf3);

    auto odom2 = createOdometryFactor(3, 2, 3, Eigen::Vector3d(1, 0, 0));
    graph_.addFactor(odom2);
    store_.addFactor(odom2);

    // Second optimization
    EXPECT_TRUE(graph_.optimize());

    // Save and verify incremental updates
    const std::string test_file = "/tmp/test_incremental_map.pb";
    EXPECT_TRUE(store_.save(test_file));

    core::storage::MapStore loaded_store;
    EXPECT_TRUE(loaded_store.load(test_file));

    // Verify all keyframes and factors are present
    EXPECT_EQ(loaded_store.getAllKeyFrames().size(), 3);
    EXPECT_EQ(loaded_store.getAllFactors().size(), 3);

    // Verify connected factors for middle keyframe
    auto connected_factors = loaded_store.getConnectedFactors(2);
    EXPECT_EQ(connected_factors.size(), 2);  // Should be connected to both odom factors
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}