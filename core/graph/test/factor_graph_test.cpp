#include <gtest/gtest.h>
#include <core/graph/factor_graph.hpp>
#include <core/graph/keyframe_manager.hpp>
#include <core/graph/util.hpp>

namespace core {
namespace graph {
namespace testing {

class FactorGraphTest : public ::testing::Test {
protected:
    void SetUp() override {
        graph_ = std::make_unique<FactorGraph>();
    }

    types::Pose createTestPose(double x, double y, double z) {
        types::Pose pose;
        pose.position = Eigen::Vector3d(x, y, z);
        pose.orientation = Eigen::Quaterniond::Identity();
        return pose;
    }

    std::unique_ptr<FactorGraph> graph_;
};

TEST_F(FactorGraphTest, AddKeyFrame) {
    auto kf = std::make_shared<types::KeyFrame>();
    kf->id = 1;
    kf->pose = createTestPose(1, 0, 0);

    graph_->addKeyFrame(kf);
    auto retrieved = graph_->getKeyFramePtr(1);

    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->id, kf->id);
    EXPECT_NEAR((retrieved->pose.position - kf->pose.position).norm(), 0, 1e-9);
}

TEST_F(FactorGraphTest, SimpleOptimization) {
    // Add three keyframes in a triangle
    auto kf1 = std::make_shared<types::KeyFrame>();
    auto kf2 = std::make_shared<types::KeyFrame>();
    auto kf3 = std::make_shared<types::KeyFrame>();

    kf1->id = 1;
    kf2->id = 2;
    kf3->id = 3;

    kf1->pose = createTestPose(0, 0, 0);
    kf2->pose = createTestPose(1, 0, 0);
    kf3->pose = createTestPose(1, 1, 0);

    graph_->addKeyFrame(kf1);
    graph_->addKeyFrame(kf2);
    graph_->addKeyFrame(kf3);

    // Add odometry factors
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

    types::Factor odom1;
    odom1.type = proto::FactorType::ODOMETRY;
    odom1.connected_nodes = {1, 2};
    odom1.measurement.emplace<0>(createTestPose(1, 0, 0));
    odom1.information = information;

    types::Factor odom2;
    odom2.type = proto::FactorType::ODOMETRY;
    odom2.connected_nodes = {2, 3};
    odom2.measurement.emplace<0>(createTestPose(0, 1, 0));
    odom2.information = information;

    types::Factor loop;
    loop.type = proto::FactorType::LOOP_CLOSURE;
    loop.connected_nodes = {3, 1};
    loop.measurement.emplace<0>(createTestPose(-1, -1, 0));
    loop.information = information;

    graph_->addFactor(odom1);
    graph_->addFactor(odom2);
    graph_->addFactor(loop);

    EXPECT_TRUE(graph_->optimize());
}

TEST_F(FactorGraphTest, DumpToVTK) {
    // Create a simple graph
    auto kf1 = std::make_shared<types::KeyFrame>();
    auto kf2 = std::make_shared<types::KeyFrame>();
    kf1->id = 1;
    kf2->id = 2;
    kf1->pose = createTestPose(0, 0, 0);
    kf2->pose = createTestPose(1, 0, 0);

    graph_->addKeyFrame(kf1);
    graph_->addKeyFrame(kf2);

    types::Factor odom;
    odom.type = proto::FactorType::ODOMETRY;
    odom.connected_nodes = {1, 2};
    odom.measurement.emplace<0>(createTestPose(1, 0, 0));
    odom.information = Eigen::Matrix<double, 6, 6>::Identity();

    graph_->addFactor(odom);

    // Dump to VTK
    EXPECT_NO_THROW(util::dumpFactorGraph(*graph_, "test_graph.vtk"));

    // Verify file exists and has content
    std::ifstream file("test_graph.vtk");
    EXPECT_TRUE(file.good());
    std::string line;
    std::getline(file, line);
    EXPECT_EQ(line, "# vtk DataFile Version 3.0");
}

TEST_F(FactorGraphTest, ImuFactor) {
    // Create two keyframes
    auto kf1 = std::make_shared<types::KeyFrame>();
    auto kf2 = std::make_shared<types::KeyFrame>();
    kf1->id = 1;
    kf2->id = 2;
    kf1->pose = createTestPose(0, 0, 0);
    kf2->pose = createTestPose(1, 0, 0);

    graph_->addKeyFrame(kf1);
    graph_->addKeyFrame(kf2);

    // Create an IMU preintegrated measurement
    types::ImuPreintegratedMeasurement imu_measurement;
    imu_measurement.preintegrated_position = Eigen::Vector3d(1.0, 0.0, 0.0);
    imu_measurement.preintegrated_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    imu_measurement.preintegrated_rotation = Eigen::Quaterniond::Identity();
    imu_measurement.delta_time = 1.0;
    imu_measurement.accelerometer_bias = Eigen::Vector3d::Zero();
    imu_measurement.gyroscope_bias = Eigen::Vector3d::Zero();
    imu_measurement.covariance_matrix = Eigen::Matrix<double, 9, 9>::Identity() * 0.01;

    // Create IMU factor
    types::Factor imu_factor;
    imu_factor.type = proto::FactorType::IMU_PREINTEGRATED;
    imu_factor.connected_nodes = {1, 2};
    imu_factor.measurement.emplace<2>(imu_measurement);
    imu_factor.information = Eigen::MatrixXd::Identity(9, 9) * 10.0;

    // Add factor to graph
    EXPECT_NO_THROW(graph_->addFactor(imu_factor));

    // Verify factor was added
    EXPECT_TRUE(graph_->optimize());
}

TEST_F(FactorGraphTest, EnhancedKeyframeDecisions) {
    // Create KeyframeManager for testing
    KeyframeThresholds thresholds;
    thresholds.distance_threshold = 0.1;  // 10cm
    thresholds.rotation_threshold = 10.0; // 10 degrees
    thresholds.use_imu_motion = true;
    thresholds.max_linear_velocity = 1.0; // 1 m/s
    thresholds.max_angular_velocity = 0.5; // 0.5 rad/s
    thresholds.max_acceleration = 15.0;    // 15 m/s² (well above combined gravity + acceleration)

    KeyframeManager manager(thresholds);

    // Test 1: First keyframe should always be created
    auto pose1 = createTestPose(0, 0, 0);
    pose1.timestamp = 1.0;

    types::ImuData imu1;
    imu1.linear_acceleration = Eigen::Vector3d(0, 0, -9.81); // gravity only
    imu1.angular_velocity = Eigen::Vector3d::Zero();

    EXPECT_TRUE(manager.shouldCreateKeyframe(pose1, std::nullopt, std::nullopt, imu1));
    EXPECT_EQ(manager.getLastDecisionReason(), "First keyframe");

    manager.updateLastKeyframe(pose1, std::nullopt, std::nullopt, imu1);

    // Test 2: Small motion with normal gravity should not create keyframe
    auto pose2 = createTestPose(0.05, 0, 0); // 5cm movement
    pose2.timestamp = 2.0;

    types::ImuData imu2;
    imu2.linear_acceleration = Eigen::Vector3d(0, 0, -9.81); // gravity only
    imu2.angular_velocity = Eigen::Vector3d::Zero();

    EXPECT_FALSE(manager.shouldCreateKeyframe(pose2, std::nullopt, std::nullopt, imu2));

    // Update for next test so we have last_imu_data_
    manager.updateLastKeyframe(pose2, std::nullopt, std::nullopt, imu2);

    // Test 3: High angular velocity should create keyframe
    auto pose3 = createTestPose(0.06, 0, 0); // Still small movement but different position
    pose3.timestamp = 3.0;

    types::ImuData imu3;
    imu3.linear_acceleration = Eigen::Vector3d(0, 0, -9.81); // gravity only
    imu3.angular_velocity = Eigen::Vector3d(0, 0, 1.0); // High angular velocity!

    EXPECT_TRUE(manager.shouldCreateKeyframe(pose3, std::nullopt, std::nullopt, imu3));

    // Update state for next test
    manager.updateLastKeyframe(pose3, std::nullopt, std::nullopt, imu3);

    // Test 4: Large distance should create keyframe regardless of IMU
    auto pose4 = createTestPose(1.0, 0, 0); // 1m movement - exceeds distance threshold
    pose4.timestamp = 4.0;

    types::ImuData imu4;
    imu4.linear_acceleration = Eigen::Vector3d(0, 0, -9.81); // gravity only
    imu4.angular_velocity = Eigen::Vector3d::Zero();

    EXPECT_TRUE(manager.shouldCreateKeyframe(pose4, std::nullopt, std::nullopt, imu4));
}

}  // namespace testing
}  // namespace graph
}  // namespace core
