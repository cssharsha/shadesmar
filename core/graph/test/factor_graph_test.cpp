#include <gtest/gtest.h>
#include <core/graph/factor_graph.hpp>
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
    auto retrieved = graph_->getKeyFrame(1);

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

}  // namespace testing
}  // namespace graph
}  // namespace core
