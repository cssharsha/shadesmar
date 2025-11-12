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

        options.callbacks.onClientAdvertise = [this](uint32_t client_id,
                                                     const foxglove::ClientChannel& channel) {
            LOG(INFO) << "[FoxgloveServer] Client " << client_id << " advertised channel " << channel.id
                      << " on topic '" << channel.topic << "'";
            std::lock_guard<std::mutex> lock(this->subscription_mutex_);
            this->client_channel_to_topic_[channel.id] = std::string(channel.topic);
        };

        options.callbacks.onClientUnadvertise = [this](uint32_t chanId, uint32_t clientId) {
            LOG(INFO) << "[FoxgloveServer] Client " << clientId << " unadvertised channel " << chanId;
            std::lock_guard<std::mutex> lock(this->subscription_mutex_);
            this->client_channel_to_topic_.erase(chanId);
        };

        options.callbacks.onMessageData = [this](uint32_t client_id, uint32_t client_channel_id,
                                                 const std::byte* data, size_t data_len) {
            this->onMessageData(client_id, client_channel_id, data, data_len);
        };

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

    channels_.clear();

    if (server_) {
        if (!is_exiting.load()) {
            try {
                server_->stop();
            } catch (...) {
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
    auto it = channels_.find(topic);
    if (it != channels_.end()) {
        return it->second;
    }

    auto schema_str = foxglove_schemas::getSchema(schema_name);
    std::optional<foxglove::Schema> schema;
    if (schema_str.has_value()) {
        schema_storage_[topic] = *schema_str;
        foxglove::Schema s;
        s.name = schema_name;
        s.encoding = "jsonschema";
        s.data = reinterpret_cast<const std::byte*>(schema_storage_[topic].data());
        s.data_len = schema_storage_[topic].size();
        schema = s;
    } else {
        LOG(WARNING) << "[FoxgloveServer] Failed to load schema: " << schema_name;
    }

    auto result = foxglove::RawChannel::create(topic, "json", schema, context_);
    if (!result) {
        throw std::runtime_error("Failed to create channel for topic: " + topic);
    }

    auto [inserted_it, success] = channels_.emplace(topic, std::move(*result));
    return inserted_it->second;
}

void FoxgloveServer::publishPointCloud(const std::string& topic, const core::types::PointCloud& cloud,
                                       uint64_t timestamp_ns) {
    if (!running_) return;

    try {
        json msg;
        msg["timestamp"]["sec"] = timestamp_ns / 1000000000ULL;
        msg["timestamp"]["nsec"] = timestamp_ns % 1000000000ULL;
        msg["frame_id"] = cloud.frame_id;
        msg["pose"]["position"]["x"] = 0.0;
        msg["pose"]["position"]["y"] = 0.0;
        msg["pose"]["position"]["z"] = 0.0;
        msg["pose"]["orientation"]["x"] = 0.0;
        msg["pose"]["orientation"]["y"] = 0.0;
        msg["pose"]["orientation"]["z"] = 0.0;
        msg["pose"]["orientation"]["w"] = 1.0;

        size_t num_points = cloud.points.size();
        json fields = json::array();
        fields.push_back({{"name", "x"}, {"offset", 0}, {"type", 7}});
        fields.push_back({{"name", "y"}, {"offset", 4}, {"type", 7}});
        fields.push_back({{"name", "z"}, {"offset", 8}, {"type", 7}});
        fields.push_back({{"name", "red"}, {"offset", 12}, {"type", 1}});
        fields.push_back({{"name", "green"}, {"offset", 13}, {"type", 1}});
        fields.push_back({{"name", "blue"}, {"offset", 14}, {"type", 1}});
        fields.push_back({{"name", "alpha"}, {"offset", 15}, {"type", 1}});

        size_t point_stride = 16;
        msg["fields"] = fields;
        msg["point_stride"] = point_stride;

        std::vector<uint8_t> binary_data(point_stride * num_points);
        for (size_t i = 0; i < num_points; ++i) {
            size_t offset = i * point_stride;

            // Copy position (convert from Eigen::Vector3d to float)
            float pos[3] = {
                static_cast<float>(cloud.points[i].x()),
                static_cast<float>(cloud.points[i].y()),
                static_cast<float>(cloud.points[i].z())
            };
            std::memcpy(&binary_data[offset], pos, 12);

            // Copy color (convert from Eigen::Vector3d [0-1] to uint8_t [0-255])
            if (i < cloud.colors.size()) {
                binary_data[offset + 12] = static_cast<uint8_t>(std::clamp(cloud.colors[i].x(), 0.0, 1.0) * 255);
                binary_data[offset + 13] = static_cast<uint8_t>(std::clamp(cloud.colors[i].y(), 0.0, 1.0) * 255);
                binary_data[offset + 14] = static_cast<uint8_t>(std::clamp(cloud.colors[i].z(), 0.0, 1.0) * 255);
                binary_data[offset + 15] = 255;  // alpha
            } else {
                binary_data[offset + 12] = 255;
                binary_data[offset + 13] = 255;
                binary_data[offset + 14] = 255;
                binary_data[offset + 15] = 255;
            }
        }

        msg["data"] = base64Encode(binary_data);
        std::string msg_str = msg.dump();
        auto& channel = getOrCreateChannel(topic, "foxglove.PointCloud");
        channel.log(reinterpret_cast<const std::byte*>(msg_str.data()), msg_str.size(), timestamp_ns);

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to publish point cloud: " << e.what();
    }
}

void FoxgloveServer::publishImage(const std::string& topic, const core::types::Image& image,
                                  uint64_t timestamp_ns) {
    if (!running_ || image.data.empty()) return;

    try {
        std::vector<uint8_t> encoded_image;
        if (!cv::imencode(".jpg", image.data, encoded_image, {cv::IMWRITE_JPEG_QUALITY, 90})) {
            LOG(ERROR) << "[FoxgloveServer] Failed to encode image as JPEG";
            return;
        }

        json msg;
        msg["timestamp"]["sec"] = timestamp_ns / 1000000000ULL;
        msg["timestamp"]["nsec"] = timestamp_ns % 1000000000ULL;
        msg["frame_id"] = image.frame_id;
        msg["format"] = "jpeg";
        msg["data"] = base64Encode(encoded_image);

        std::string msg_str = msg.dump();
        auto& channel = getOrCreateChannel(topic, "foxglove.CompressedImage");
        channel.log(reinterpret_cast<const std::byte*>(msg_str.data()), msg_str.size(), timestamp_ns);

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to publish image: " << e.what();
    }
}

void FoxgloveServer::publishKeyframePoses(const std::string& topic,
                                          const std::vector<core::types::Pose>& poses,
                                          uint64_t timestamp_ns) {
    if (!running_ || poses.empty()) return;

    try {
        json msg;
        msg["timestamp"]["sec"] = timestamp_ns / 1000000000ULL;
        msg["timestamp"]["nsec"] = timestamp_ns % 1000000000ULL;
        msg["frame_id"] = poses[0].frame_id;

        json poses_array = json::array();
        for (const auto& pose : poses) {
            json pose_json;
            pose_json["position"]["x"] = pose.position.x();
            pose_json["position"]["y"] = pose.position.y();
            pose_json["position"]["z"] = pose.position.z();
            pose_json["orientation"]["x"] = pose.orientation.x();
            pose_json["orientation"]["y"] = pose.orientation.y();
            pose_json["orientation"]["z"] = pose.orientation.z();
            pose_json["orientation"]["w"] = pose.orientation.w();
            poses_array.push_back(pose_json);
        }
        msg["poses"] = poses_array;

        std::string msg_str = msg.dump();
        auto& channel = getOrCreateChannel(topic, "foxglove.PosesInFrame");
        channel.log(reinterpret_cast<const std::byte*>(msg_str.data()), msg_str.size(), timestamp_ns);

        LOG(INFO) << "[FoxgloveServer] Published " << poses.size() << " keyframe poses to " << topic;
        LOG(INFO) << "[FoxgloveServer] Sample message structure: " << msg.dump(2).substr(0, 500);

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to publish keyframe poses: " << e.what();
    }
}

uint32_t FoxgloveServer::subscribeToCameraPose(const std::string& topic, PoseCallback callback) {
    if (!running_) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    pose_callback_ = callback;
    camera_pose_topic_ = topic;
    LOG(INFO) << "[FoxgloveServer] Registered callback for camera pose topic: " << topic;
    return 1;
}

uint32_t FoxgloveServer::subscribeToCameraInfo(const std::string& topic, CameraInfoCallback callback) {
    if (!running_) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    camera_info_callback_ = callback;
    camera_info_topic_ = topic;
    LOG(INFO) << "[FoxgloveServer] Registered callback for camera info topic: " << topic;
    return 1;
}

void FoxgloveServer::onMessageData(uint32_t client_id, uint32_t client_channel_id,
                                   const std::byte* data, size_t data_len) {
    std::string topic;
    {
        std::lock_guard<std::mutex> lock(subscription_mutex_);
        auto it = client_channel_to_topic_.find(client_channel_id);
        if (it == client_channel_to_topic_.end()) {
            LOG(WARNING) << "[FoxgloveServer] Received message on unknown channel " << client_channel_id;
            return;
        }
        topic = it->second;
    }

    LOG(INFO) << "[FoxgloveServer] onMessageData: " << data_len << " bytes from client "
              << client_id << " on topic '" << topic << "'";

    try {
        std::string json_str(reinterpret_cast<const char*>(data), data_len);
        json msg = json::parse(json_str);

        if (topic == camera_pose_topic_ && pose_callback_) {
            core::types::Pose pose;
            const auto& pose_msg = msg["pose"];
            pose.position.x() = pose_msg["position"]["x"].get<double>();
            pose.position.y() = pose_msg["position"]["y"].get<double>();
            pose.position.z() = pose_msg["position"]["z"].get<double>();
            pose.orientation.w() = pose_msg["orientation"]["w"].get<double>();
            pose.orientation.x() = pose_msg["orientation"]["x"].get<double>();
            pose.orientation.y() = pose_msg["orientation"]["y"].get<double>();
            pose.orientation.z() = pose_msg["orientation"]["z"].get<double>();

            const auto& header = msg["header"];
            pose.frame_id = header["frame_id"].get<std::string>();
            uint64_t sec = header["stamp"]["sec"].get<uint64_t>();
            uint64_t nsec = header["stamp"]["nsec"].get<uint64_t>();
            pose.timestamp = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;

            LOG(INFO) << "[FoxgloveServer] Received camera pose: position=["
                      << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
                      << "], orientation=[w=" << pose.orientation.w()
                      << ", x=" << pose.orientation.x() << ", y=" << pose.orientation.y()
                      << ", z=" << pose.orientation.z() << "], frame_id=" << pose.frame_id
                      << ", timestamp=" << pose.timestamp;

            pose_callback_(pose);

        } else if (topic == camera_info_topic_ && camera_info_callback_) {
            core::types::CameraInfo info;
            const auto& header = msg["header"];
            info.frame_id = header["frame_id"].get<std::string>();
            info.height = msg["height"].get<uint32_t>();
            info.width = msg["width"].get<uint32_t>();

            // K is the 3x3 camera matrix
            std::vector<double> K_vec = msg["K"].get<std::vector<double>>();
            if (K_vec.size() == 9) {
                info.k = K_vec;
            }

            // P is the 3x4 projection matrix
            std::vector<double> P_vec = msg["P"].get<std::vector<double>>();
            if (P_vec.size() == 12) {
                info.p = P_vec;
            }

            // D is the distortion coefficients
            if (msg.contains("D")) {
                info.d = msg["D"].get<std::vector<double>>();
            }

            if (msg.contains("distortion_model")) {
                info.distortion_model = msg["distortion_model"].get<std::string>();
            }

            LOG(INFO) << "[FoxgloveServer] Received camera info: width=" << info.width
                      << ", height=" << info.height << ", frame_id=" << info.frame_id
                      << ", K=[" << (info.k.size() >= 9 ?
                         std::to_string(info.k[0]) + ", " + std::to_string(info.k[4]) + ", fx/fy" : "empty")
                      << "], distortion_model=" << info.distortion_model;

            camera_info_callback_(info);
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "[FoxgloveServer] Failed to parse message on topic '" << topic
                   << "': " << e.what();
    }
}

}  // namespace viz
