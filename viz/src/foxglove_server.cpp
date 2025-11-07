#include "viz/foxglove_server.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>

#include "common/logging/logging.hpp"
#include "viz/foxglove_schemas.hpp"

using json = nlohmann::json;

namespace viz {

// Static flag to track if we're in process exit
static std::atomic<bool> is_exiting{false};

// Register atexit handler to set flag
static void markExiting() {
    is_exiting.store(true);
}

static bool registerExitHandler() {
    std::atexit(markExiting);
    return true;
}

static bool exit_handler_registered = registerExitHandler();

FoxgloveServer::FoxgloveServer(const std::string& host, uint16_t port)
    : host_(host), port_(port), running_(false) {}

FoxgloveServer::~FoxgloveServer() {
    shutdown();
}

bool FoxgloveServer::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (running_) {
        LOG(WARNING) << "[FoxgloveServer] Already running";
        return false;
    }

    try {
        foxglove::WebSocketServerOptions options;
        options.context = context_;
        options.name = "Shadesmar Visualization Server";
        options.host = host_;
        options.port = port_;
        options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;

        // Set up message data callback for receiving camera poses
        options.callbacks.onMessageData = [this](uint32_t client_id, uint32_t client_channel_id,
                                                 const std::byte* data, size_t data_len) {
            this->onMessageData(client_id, client_channel_id, data, data_len);
        };

        // Add subscription callbacks to see if clients are subscribing
        options.callbacks.onSubscribe = [](uint64_t channel_id,
                                           const foxglove::ClientMetadata& client) {
            LOG(INFO) << "[FoxgloveServer] Client " << client.id << " subscribed to channel "
                      << channel_id;
        };

        options.callbacks.onUnsubscribe = [](uint64_t channel_id,
                                             const foxglove::ClientMetadata& client) {
            LOG(INFO) << "[FoxgloveServer] Client " << client.id << " unsubscribed from channel "
                      << channel_id;
        };

        auto result = foxglove::WebSocketServer::create(std::move(options));
        if (!result) {
            LOG(ERROR) << "[FoxgloveServer] Failed to create server";
            return false;
        }

        server_ = std::make_unique<foxglove::WebSocketServer>(std::move(*result));
        running_ = true;
        LOG(INFO) << "[FoxgloveServer] Started on " << host_ << ":" << server_->port();
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to initialize: " << e.what();
        return false;
    }
}

void FoxgloveServer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!running_) {
        return;
    }

    // Close all channels
    channels_.clear();

    if (server_) {
        // During process exit, Rust thread-local storage may already be destroyed
        // causing panics in the Foxglove SDK. Skip stop() if we're in the exit phase.
        if (!is_exiting.load()) {
            try {
                server_->stop();
            } catch (...) {
                // Ignore any exceptions during shutdown
                LOG(WARNING) << "[FoxgloveServer] Exception during server stop";
            }
        } else {
            LOG(INFO) << "[FoxgloveServer] Skipping server stop during process exit";
        }
        server_.reset();
    }

    running_ = false;
    LOG(INFO) << "[FoxgloveServer] Shutdown complete";
}

bool FoxgloveServer::isRunning() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_;
}

std::string FoxgloveServer::base64Encode(const std::vector<uint8_t>& data) {
    static const char base64_chars[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string encoded;
    size_t i = 0;
    uint8_t char_array_3[3];
    uint8_t char_array_4[4];

    for (uint8_t byte : data) {
        char_array_3[i++] = byte;
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; i < 4; i++)
                encoded += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for (size_t j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

        for (size_t j = 0; j < i + 1; j++)
            encoded += base64_chars[char_array_4[j]];

        while (i++ < 3)
            encoded += '=';
    }

    return encoded;
}

foxglove::RawChannel& FoxgloveServer::getOrCreateChannel(const std::string& topic,
                                                         const std::string& schema_name) {
    // Check if channel already exists
    auto it = channels_.find(topic);
    if (it != channels_.end()) {
        return it->second;
    }

    // Get schema string from registry and store it
    auto schema_str = foxglove_schemas::getSchema(schema_name);

    // Create foxglove::Schema if schema string exists
    std::optional<foxglove::Schema> schema;
    if (schema_str.has_value()) {
        // Store schema string to keep it alive
        schema_storage_[topic] = *schema_str;

        foxglove::Schema s;
        s.name = schema_name;
        s.encoding = "jsonschema";
        s.data = reinterpret_cast<const std::byte*>(schema_storage_[topic].data());
        s.data_len = schema_storage_[topic].size();
        schema = s;

        LOG(INFO) << "[FoxgloveServer] Loaded schema: " << schema_name
                  << " size=" << schema_storage_[topic].size() << " bytes";
    } else {
        LOG(WARNING) << "[FoxgloveServer] Failed to load schema: " << schema_name;
    }

    LOG(INFO) << "[FoxgloveServer] Creating channel: topic=" << topic << " schema=" << schema_name
              << " has_schema=" << schema.has_value();

    // Create new channel with schema
    auto result = foxglove::RawChannel::create(topic, "json", schema, context_);
    if (!result) {
        throw std::runtime_error("Failed to create channel for topic: " + topic);
    }

    auto [inserted_it, success] = channels_.emplace(topic, std::move(*result));
    return inserted_it->second;
}

void FoxgloveServer::publishPointCloud(const std::string& topic, const PointCloudData& data,
                                       uint64_t timestamp_ns) {
    LOG(INFO) << "[FoxgloveServer] publishPointCloud called: topic=" << topic
              << " running=" << running_ << " positions=" << data.positions.size();

    if (!running_) {
        LOG(WARNING) << "[FoxgloveServer] Not running, skipping publish";
        return;
    }

    try {
        // Create foxglove.PointCloud message
        json msg;
        msg["timestamp"]["sec"] = timestamp_ns / 1000000000ULL;
        msg["timestamp"]["nsec"] = timestamp_ns % 1000000000ULL;
        msg["frame_id"] = data.frame_id;

        // Pose (origin with identity orientation)
        msg["pose"]["position"]["x"] = 0.0;
        msg["pose"]["position"]["y"] = 0.0;
        msg["pose"]["position"]["z"] = 0.0;
        msg["pose"]["orientation"]["x"] = 0.0;
        msg["pose"]["orientation"]["y"] = 0.0;
        msg["pose"]["orientation"]["z"] = 0.0;
        msg["pose"]["orientation"]["w"] = 1.0;

        // Calculate number of points
        size_t num_points = data.positions.size() / 3;
        LOG(INFO) << "[FoxgloveServer] Publishing " << num_points << " points to " << topic;

        // Define fields (foxglove.PackedElementField format)
        // foxglove.NumericType: FLOAT32 = 7, UINT8 = 1
        json fields = json::array();

        // Position fields (x, y, z) - required
        fields.push_back({
            {"name", "x"}, {"offset", 0}, {"type", 7}  // FLOAT32 = 7
        });
        fields.push_back({{"name", "y"}, {"offset", 4}, {"type", 7}});
        fields.push_back({{"name", "z"}, {"offset", 8}, {"type", 7}});

        // Color fields (red, green, blue, alpha) - optional
        fields.push_back({
            {"name", "red"}, {"offset", 12}, {"type", 1}  // UINT8 = 1
        });
        fields.push_back({{"name", "green"}, {"offset", 13}, {"type", 1}});
        fields.push_back({{"name", "blue"}, {"offset", 14}, {"type", 1}});
        fields.push_back({{"name", "alpha"}, {"offset", 15}, {"type", 1}});

        // Optional radius field
        size_t point_stride = 16;  // x,y,z (12 bytes) + rgba (4 bytes)
        if (!data.radii.empty()) {
            fields.push_back({
                {"name", "radius"}, {"offset", 16}, {"type", 7}  // FLOAT32
            });
            point_stride = 20;  // Add 4 bytes for radius
        }

        msg["fields"] = fields;
        msg["point_stride"] = point_stride;

        // Pack binary data
        std::vector<uint8_t> binary_data(point_stride * num_points);

        for (size_t i = 0; i < num_points; ++i) {
            size_t offset = i * point_stride;

            // Position (x, y, z) as FLOAT32
            std::memcpy(&binary_data[offset], &data.positions[i * 3], 12);

            // Color (red, green, blue, alpha) as separate UINT8
            if (i * 4 + 3 < data.colors.size()) {
                binary_data[offset + 12] = data.colors[i * 4 + 0];  // red
                binary_data[offset + 13] = data.colors[i * 4 + 1];  // green
                binary_data[offset + 14] = data.colors[i * 4 + 2];  // blue
                binary_data[offset + 15] = data.colors[i * 4 + 3];  // alpha
            } else {
                binary_data[offset + 12] = 255;  // default white
                binary_data[offset + 13] = 255;
                binary_data[offset + 14] = 255;
                binary_data[offset + 15] = 255;
            }

            // Optional radius as FLOAT32
            if (!data.radii.empty() && i < data.radii.size()) {
                std::memcpy(&binary_data[offset + 16], &data.radii[i], 4);
            }
        }

        msg["data"] = base64Encode(binary_data);

        // Serialize and send
        std::string msg_str = msg.dump();
        LOG(INFO) << "[FoxgloveServer] Message size: " << msg_str.size() << " bytes";

        // Debug: log first message structure (before the large data field)
        static bool logged_first = false;
        if (!logged_first) {
            // Find where "data" field starts
            size_t data_pos = msg_str.find("\"data\":\"");
            std::string preview =
                msg_str.substr(0, data_pos != std::string::npos ? data_pos + 20 : 1000);
            LOG(INFO) << "[FoxgloveServer] First message structure: " << preview;
            logged_first = true;
        }

        auto& channel = getOrCreateChannel(topic, "foxglove.PointCloud");
        LOG(INFO) << "[FoxgloveServer] Got channel, calling log()";
        channel.log(reinterpret_cast<const std::byte*>(msg_str.data()), msg_str.size(),
                    timestamp_ns);
        LOG(INFO) << "[FoxgloveServer] Successfully published point cloud to " << topic;

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to publish point cloud: " << e.what();
    }
}

void FoxgloveServer::publishImage(const std::string& topic, const core::types::Image& image,
                                  uint64_t timestamp_ns) {
    if (!running_) {
        LOG(WARNING) << "[FoxgloveServer] Not running, cannot publish image";
        return;
    }

    if (image.data.empty()) {
        LOG(WARNING) << "[FoxgloveServer] Image data is empty, cannot publish";
        return;
    }

    LOG(INFO) << "[FoxgloveServer] publishImage: image size=" << image.data.rows << "x"
              << image.data.cols << " channels=" << image.data.channels()
              << " encoding=" << image.encoding;

    try {
        // Encode image as JPEG for efficient transport
        std::vector<uint8_t> encoded_image;
        bool success =
            cv::imencode(".jpg", image.data, encoded_image, {cv::IMWRITE_JPEG_QUALITY, 90});

        if (!success || encoded_image.empty()) {
            LOG(ERROR) << "[FoxgloveServer] Failed to encode image as JPEG";
            return;
        }

        LOG(INFO) << "[FoxgloveServer] Encoded image to JPEG: " << encoded_image.size() << " bytes";

        // Create foxglove.CompressedImage message
        json msg;
        msg["timestamp"]["sec"] = timestamp_ns / 1000000000ULL;
        msg["timestamp"]["nsec"] = timestamp_ns % 1000000000ULL;
        msg["frame_id"] = image.frame_id;
        msg["format"] = "jpeg";
        msg["data"] = base64Encode(encoded_image);

        // Serialize and send
        std::string msg_str = msg.dump();
        LOG(INFO) << "[FoxgloveServer] Message size: " << msg_str.size() << " bytes";

        auto& channel = getOrCreateChannel(topic, "foxglove.CompressedImage");
        channel.log(reinterpret_cast<const std::byte*>(msg_str.data()), msg_str.size(),
                    timestamp_ns);

        LOG(INFO) << "[FoxgloveServer] Successfully published image to " << topic;

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to publish image: " << e.what();
    }
}

uint32_t FoxgloveServer::subscribeToCameraPose(const std::string& topic, PoseCallback callback) {
    if (!running_) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Store the callback
    pose_callback_ = callback;

    LOG(INFO) << "[FoxgloveServer] Subscribed to " << topic << " for camera poses";

    // Note: With Foxglove SDK, client channels are handled through onClientAdvertise callback
    // The actual subscription happens when the client advertises a channel
    return 1;  // Return non-zero to indicate success
}

void FoxgloveServer::onMessageData(uint32_t client_id, uint32_t client_channel_id,
                                   const std::byte* data, size_t data_len) {
    LOG(INFO) << "[FoxgloveServer] onMessageData called " << data_len << " bytes with client id "
              << client_id << " at channel: " << client_channel_id;

    if (!pose_callback_) {
        LOG(WARNING) << "[FoxgloveServer] No pose callback registered, ignoring message";
        return;
    }

    try {
        // Parse JSON message - Foxglove client publishes in JSON format
        std::string json_str(reinterpret_cast<const char*>(data), data_len);

        // Log first 200 characters for debugging
        LOG(INFO) << "[FoxgloveServer] Received data preview: "
                  << json_str.substr(0, std::min(size_t(200), json_str.size()));

        json msg = json::parse(json_str);

        // Extract pose from geometry_msgs/PoseStamped
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        pose.orientation = Eigen::Quaterniond::Identity();
        pose.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;

        if (msg.contains("pose")) {
            const auto& pose_msg = msg["pose"];

            if (pose_msg.contains("position")) {
                pose.position.x() = pose_msg["position"]["x"].get<double>();
                pose.position.y() = pose_msg["position"]["y"].get<double>();
                pose.position.z() = pose_msg["position"]["z"].get<double>();
            }

            if (pose_msg.contains("orientation")) {
                pose.orientation.w() = pose_msg["orientation"]["w"].get<double>();
                pose.orientation.x() = pose_msg["orientation"]["x"].get<double>();
                pose.orientation.y() = pose_msg["orientation"]["y"].get<double>();
                pose.orientation.z() = pose_msg["orientation"]["z"].get<double>();
            }
        }

        if (msg.contains("header")) {
            const auto& header = msg["header"];
            if (header.contains("frame_id")) {
                pose.frame_id = header["frame_id"].get<std::string>();
            }
            if (header.contains("stamp")) {
                uint64_t sec = header["stamp"]["sec"].get<uint64_t>();
                uint64_t nsec = header["stamp"]["nsec"].get<uint64_t>();
                pose.timestamp = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
            }
        }

        LOG(INFO) << "[FoxgloveServer] Parsed pose: position=[" << pose.position.transpose()
                  << "] orientation=[" << pose.orientation.w() << "," << pose.orientation.x()
                  << "," << pose.orientation.y() << "," << pose.orientation.z() << "]";

        // Invoke callback
        pose_callback_(pose);

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to parse pose message: " << e.what();
    }
}

}  // namespace viz
