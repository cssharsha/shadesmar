#include "training_gs_processor.hpp"
#include <gtest/gtest.h>
#include <memory>

namespace gaussian_splatting {

class TrainingGaussianSplatProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.map_base_path = "/tmp/test_map";
        config_.init_mode = SplatInitMode::RANDOM_POINTS;
        config_.keyframes_per_batch = 5;
        config_.device = torch::kCPU;  // Use CPU for testing
    }
    
    TrainingProcessorConfig<rendering::BilateralGrid,
                           optimization::DensificationController,
                           training::TrainingConfig,
                           rendering::DifferentiableRasterizer> config_;
};

TEST_F(TrainingGaussianSplatProcessorTest, ConstructorTest) {
    // Test basic construction
    EXPECT_NO_THROW({
        StandardTrainingProcessor processor(config_);
    });
}

TEST_F(TrainingGaussianSplatProcessorTest, ConfigurationTest) {
    StandardTrainingProcessor processor(config_);
    
    // Test configuration access
    const auto& retrieved_config = processor.getConfig();
    EXPECT_EQ(retrieved_config.map_base_path, config_.map_base_path);
    EXPECT_EQ(retrieved_config.init_mode, config_.init_mode);
    EXPECT_EQ(retrieved_config.keyframes_per_batch, config_.keyframes_per_batch);
}

TEST_F(TrainingGaussianSplatProcessorTest, StatsInitializationTest) {
    StandardTrainingProcessor processor(config_);
    
    // Test initial statistics
    const auto& stats = processor.getStats();
    EXPECT_EQ(stats.total_iterations.load(), 0);
    EXPECT_EQ(stats.current_batch_id.load(), 0);
    EXPECT_EQ(stats.current_loss.load(), 0.0f);
    EXPECT_EQ(stats.current_splat_count.load(), 0);
    EXPECT_FALSE(stats.is_training.load());
}

TEST_F(TrainingGaussianSplatProcessorTest, CallbackManagementTest) {
    StandardTrainingProcessor processor(config_);
    
    // Test callback setting and clearing
    bool callback_called = false;
    auto callback = [&callback_called](int iter, float loss, int count) {
        callback_called = true;
    };
    
    EXPECT_NO_THROW({
        processor.setTrainingProgressCallback(callback);
        processor.clearTrainingProgressCallback();
    });
}

TEST_F(TrainingGaussianSplatProcessorTest, TemplateInstantiationTest) {
    // Test different template instantiations compile correctly
    EXPECT_NO_THROW({
        StandardTrainingProcessor standard_processor(config_);
        CPUTrainingProcessor cpu_processor(config_);
    });
}

} // namespace gaussian_splatting