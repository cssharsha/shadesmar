#pragma once

#include <foxglove/server.hpp>
#include <mutex>
#include <string>
#include <vector>
#include <array>

#include "core/types/image.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/pose.hpp"

namespace viz {

class FoxgloveServer {
public:
    using PoseCallback = std::function<void(const core::types::Pose&)>;
    using CameraInfoCallback = std::function<void(const core::types::CameraInfo&)>;

    FoxgloveServer(const std::string& host, uint16_t port);
    ~FoxgloveServer();

    bool initialize();
    void shutdown();
    bool isRunning() const;

    void publishPointCloud(const std::string& topic, const core::types::PointCloud& cloud,
                           uint64_t timestamp_ns);
    void publishImage(const std::string& topic, const core::types::Image& image,
                      uint64_t timestamp_ns);
    void publishKeyframePoses(const std::string& topic, const std::vector<core::types::Pose>& poses,
                              uint64_t timestamp_ns);

    uint32_t subscribeToCameraPose(const std::string& topic, PoseCallback callback);
    uint32_t subscribeToCameraInfo(const std::string& topic, CameraInfoCallback callback);

private:
    std::string base64Encode(const std::vector<uint8_t>& data);
    foxglove::RawChannel& getOrCreateChannel(const std::string& topic,
                                             const std::string& schema_name);
    void onMessageData(uint32_t client_id, uint32_t client_channel_id, const std::byte* data,
                       size_t data_len);

    std::string host_;
    uint16_t port_;
    std::atomic<bool> running_;
    mutable std::mutex mutex_;
    std::mutex subscription_mutex_;

    foxglove::Context context_;
    std::unique_ptr<foxglove::WebSocketServer> server_;
    std::map<std::string, foxglove::RawChannel> channels_;
    std::map<std::string, std::string> schema_storage_;
    std::map<uint32_t, std::string> client_channel_to_topic_;

    PoseCallback pose_callback_;
    CameraInfoCallback camera_info_callback_;

    std::string camera_pose_topic_;
    std::string camera_info_topic_;
};

}  // namespace viz
