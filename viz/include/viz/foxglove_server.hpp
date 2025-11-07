#pragma once

// Include foxglove headers BEFORE common/logging to avoid macro conflicts
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/server.hpp>

#include "core/types/image.hpp"
#include "core/types/pose.hpp"

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace viz {

/**
 * @brief Generic Foxglove WebSocket server for publishing robotics data
 *
 * This class provides a high-level interface to the Foxglove SDK for
 * publishing standard message types (point clouds, images, poses).
 * It has no knowledge of domain-specific types like Gaussian splats.
 */
class FoxgloveServer {
public:
    /**
     * @brief Callback function for receiving camera pose messages
     * @param pose The parsed camera pose from Foxglove viewer
     */
    using PoseCallback = std::function<void(core::types::Pose& pose)>;

    /**
     * @brief Point cloud data structure for generic point cloud publishing
     */
    struct PointCloudData {
        std::vector<float> positions;  // Flat array: [x1,y1,z1, x2,y2,z2, ...]
        std::vector<uint8_t> colors;   // Flat array: [r1,g1,b1,a1, r2,g2,b2,a2, ...]
        std::vector<float> radii;      // Optional: per-point radii
        std::string frame_id;          // Coordinate frame ID
    };

    /**
     * @brief Construct a new Foxglove Server
     * @param host The host address to bind to (default: "0.0.0.0")
     * @param port The port to listen on (default: 8765)
     */
    explicit FoxgloveServer(const std::string& host = "0.0.0.0", uint16_t port = 8765);

    ~FoxgloveServer();

    // Disable copy and move
    FoxgloveServer(const FoxgloveServer&) = delete;
    FoxgloveServer& operator=(const FoxgloveServer&) = delete;
    FoxgloveServer(FoxgloveServer&&) = delete;
    FoxgloveServer& operator=(FoxgloveServer&&) = delete;

    /**
     * @brief Initialize and start the Foxglove server
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();

    /**
     * @brief Shutdown the Foxglove server
     */
    void shutdown();

    /**
     * @brief Check if the server is currently running
     * @return true if running, false otherwise
     */
    bool isRunning() const;

    /**
     * @brief Publish a point cloud to Foxglove
     * @param topic The topic name to publish to
     * @param data The point cloud data to publish
     * @param timestamp_ns Nanosecond timestamp
     */
    void publishPointCloud(const std::string& topic, const PointCloudData& data,
                           uint64_t timestamp_ns);

    /**
     * @brief Publish an image to Foxglove
     * @param topic The topic name to publish to
     * @param image The image to publish (core::types::Image)
     * @param timestamp_ns Nanosecond timestamp
     */
    void publishImage(const std::string& topic, const core::types::Image& image,
                      uint64_t timestamp_ns);

    /**
     * @brief Subscribe to camera pose updates from Foxglove viewer
     * @param topic The topic name to subscribe to
     * @param callback The callback function to invoke when pose is received
     * @return The channel ID for this subscription, or 0 on failure
     */
    uint32_t subscribeToCameraPose(const std::string& topic, PoseCallback callback);

private:
    /**
     * @brief Internal callback for receiving messages from Foxglove
     */
    void onMessageData(uint32_t client_id, uint32_t client_channel_id, const std::byte* data,
                       size_t data_len);

    /**
     * @brief Get or create a channel for publishing
     * @param topic The topic name
     * @param schema_name The schema name (e.g., "sensor_msgs/PointCloud2")
     * @return Reference to the channel
     */
    foxglove::RawChannel& getOrCreateChannel(const std::string& topic,
                                             const std::string& schema_name);

    /**
     * @brief Base64 encode binary data
     */
    std::string base64Encode(const std::vector<uint8_t>& data);

    std::string host_;
    uint16_t port_;
    std::unique_ptr<foxglove::WebSocketServer> server_;
    foxglove::Context context_;
    mutable std::mutex mutex_;
    bool running_;

    // Channel management
    std::map<std::string, foxglove::RawChannel> channels_;
    std::map<std::string, std::string> schema_storage_;  // Keeps schema strings alive
    PoseCallback pose_callback_;
};

}  // namespace viz
