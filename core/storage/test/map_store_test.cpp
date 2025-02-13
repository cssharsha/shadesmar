#include <gtest/gtest.h>
#include <filesystem>

#include "core/storage/map_store.hpp"

namespace core {
namespace storage {
namespace testing {

class MapStoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        store_ = std::make_unique<MapStore>();
        test_file_ = "test_map.pb";
    }

    void TearDown() override {
        if (std::filesystem::exists(test_file_)) {
            std::filesystem::remove(test_file_);
        }
    }

    std::unique_ptr<MapStore> store_;
    std::string test_file_;
};

TEST_F(MapStoreTest, SaveAndLoadKeyFrames) {
    auto kf = std::make_shared<types::KeyFrame>();
    kf->id = 1;
    kf->pose.position = Eigen::Vector3d(1, 2, 3);

    types::PointCloud cloud;
    cloud.points.push_back(Eigen::Vector3d(0, 0, 0));
    kf->depth_data = cloud;

    store_->addKeyFrame(kf);
    EXPECT_TRUE(store_->save(test_file_));

    auto new_store = std::make_unique<MapStore>();
    EXPECT_TRUE(new_store->load(test_file_));

    auto loaded_kf = new_store->getKeyFrame(1);
    ASSERT_NE(loaded_kf, nullptr);
    EXPECT_EQ(loaded_kf->id, kf->id);
    EXPECT_NEAR((loaded_kf->pose.position - kf->pose.position).norm(), 0, 1e-9);
}

TEST_F(MapStoreTest, SaveAndLoadFactors) {
    types::Factor factor;
    factor.id = 1;
    factor.type = proto::FactorType::ODOMETRY;
    factor.connected_nodes = {1, 2};
    factor.measurement.emplace<0>(types::Pose());
    factor.information = Eigen::Matrix<double, 6, 6>::Identity();

    store_->addFactor(factor);
    EXPECT_TRUE(store_->save(test_file_));

    auto new_store = std::make_unique<MapStore>();
    EXPECT_TRUE(new_store->load(test_file_));

    auto factors = new_store->getAllFactors();
    ASSERT_EQ(factors.size(), 1);
    EXPECT_EQ(factors[0].id, factor.id);
    EXPECT_EQ(factors[0].type, factor.type);
    EXPECT_EQ(factors[0].connected_nodes, factor.connected_nodes);
}

TEST_F(MapStoreTest, GetConnectedFactors) {
    types::Factor factor1, factor2;
    factor1.id = 1;
    factor1.connected_nodes = {1, 2};
    factor2.id = 2;
    factor2.connected_nodes = {2, 3};

    store_->addFactor(factor1);
    store_->addFactor(factor2);

    auto connected = store_->getConnectedFactors(2);
    EXPECT_EQ(connected.size(), 2);
}

}  // namespace testing
}  // namespace storage
}  // namespace core
