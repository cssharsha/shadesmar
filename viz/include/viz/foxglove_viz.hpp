#pragma once

#include <foxglove_bridge/common.hpp>
#include <foxglove_bridge/websocket_server.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include "core/graph/factor_graph.hpp"
#include "core/proto/map.pb.h"
#include "core/storage/map_store.hpp"
#include "viz/interface.hpp"

namespace viz {

// Move Config struct outside the class
struct FoxgloveVisualizerConfig {
    uint16_t port = 8765;
    std::string hostname = "localhost";
    bool use_compression = true;
};

class FoxgloveVisualizer : public VisualizerInterface {
public:
    using Config = FoxgloveVisualizerConfig;  // Type alias for backward compatibility

    explicit FoxgloveVisualizer(const Config& config = Config());
    ~FoxgloveVisualizer();

    bool initialize() override;
    bool isConnected() const override;
    void disconnect() override;

    void addPose(const core::types::Pose& pose, const std::string& label = "") override;
    void addPointCloud(const core::types::PointCloud& cloud,
                       const core::types::Pose& pose = core::types::Pose()) override;
    void addImage(const cv::Mat& image, const std::string& name = "image") override;

    void visualizeFactorGraph(const core::graph::FactorGraph& graph) override;
    void visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) override;

    void clear() override;
    void update() override;
    void setTimestamp(double timestamp) override;

    void visualizeMap(const core::storage::MapStore& map_store);
    void visualizeMapTimeline(const core::storage::MapStore& map_store, double current_time);

private:
    Config config_;
    std::unique_ptr<foxglove_bridge::WebSocketServer> server_;
    std::thread server_thread_;
    std::mutex mutex_;
    bool is_connected_ = false;
    double current_timestamp_ = 0;
    std::map<std::string, uint32_t> channel_ids_;

    // Helper functions
    void setupChannels();
    uint32_t getOrCreateChannel(const std::string& topic, const std::string& encoding,
                                const std::string& schema_name);

    // Message conversion helpers
    nlohmann::json createPoseMessage(const core::types::Pose& pose,
                                     const std::string& frame_id = "map");
    nlohmann::json createPointCloudMessage(const core::types::PointCloud& cloud,
                                           const core::types::Pose& pose);
    nlohmann::json createImageMessage(const cv::Mat& image);
};

}  // namespace viz
