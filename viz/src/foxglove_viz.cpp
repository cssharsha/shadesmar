#include "viz/foxglove_viz.hpp"
#include <foxglove_bridge/websocket_server.hpp>
#include "core/storage/map_store.hpp"

namespace viz {

FoxgloveVisualizer::FoxgloveVisualizer(const Config& config) : config_(config) {}

FoxgloveVisualizer::~FoxgloveVisualizer() {
    disconnect();
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
}

bool FoxgloveVisualizer::initialize() {
    try {
        foxglove_bridge::ServerOptions opts;
        opts.port = config_.port;
        opts.address = config_.hostname;
        opts.use_compression = config_.use_compression;

        server_ = std::make_unique<foxglove_bridge::WebSocketServer>(opts);

        // Start server in separate thread
        server_thread_ = std::thread([this]() { server_->run(); });

        setupChannels();
        is_connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize Foxglove server: " << e.what() << std::endl;
        return false;
    }
}

void FoxgloveVisualizer::setupChannels() {
    // Setup channels for different message types
    getOrCreateChannel("/poses", "foxglove::Pose", "foxglove.Pose");
    getOrCreateChannel("/point_clouds", "foxglove::PointCloud", "foxglove.PointCloud");
    getOrCreateChannel("/images", "foxglove::Image", "foxglove.RawImage");
    getOrCreateChannel("/tf", "foxglove::Transform", "foxglove.FrameTransform");
    getOrCreateChannel("/graph", "foxglove::SceneUpdate", "foxglove.SceneUpdate");
}

uint32_t FoxgloveVisualizer::getOrCreateChannel(const std::string& topic,
                                                const std::string& encoding,
                                                const std::string& schema_name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channel_ids_.find(topic);
    if (it != channel_ids_.end()) {
        return it->second;
    }

    foxglove::websocket::ChannelWithoutId channel{
        topic, encoding, schema_name,
        true  // Use binary
    };

    uint32_t channel_id = server_->addChannel(channel);
    channel_ids_[topic] = channel_id;
    return channel_id;
}

void FoxgloveVisualizer::addPose(const core::types::Pose& pose, const std::string& label) {
    if (!is_connected_)
        return;

    auto msg = createPoseMessage(pose);
    if (!label.empty()) {
        msg["label"] = label;
    }

    uint32_t channel_id = channel_ids_["/poses"];
    server_->broadcastMessage(channel_id, msg.dump());
}

void FoxgloveVisualizer::addPointCloud(const core::types::PointCloud& cloud,
                                       const core::types::Pose& pose) {
    if (!is_connected_)
        return;

    auto msg = createPointCloudMessage(cloud, pose);
    uint32_t channel_id = channel_ids_["/point_clouds"];
    server_->broadcastMessage(channel_id, msg.dump());
}

void FoxgloveVisualizer::addImage(const cv::Mat& image, const std::string& name) {
    if (!is_connected_)
        return;

    auto msg = createImageMessage(image);
    uint32_t channel_id = channel_ids_["/images"];
    server_->broadcastMessage(channel_id, msg.dump());
}

void FoxgloveVisualizer::visualizeFactorGraph(const core::graph::FactorGraph& graph) {
    if (!is_connected_)
        return;

    nlohmann::json scene;
    scene["timestamp"] = current_timestamp_;
    scene["nodes"] = nlohmann::json::array();
    scene["edges"] = nlohmann::json::array();

    // Add nodes (keyframes)
    auto keyframes = graph.getAllKeyFrames();
    for (const auto& kf : keyframes) {
        nlohmann::json node;
        node["id"] = std::to_string(kf->id);
        node["pose"] = createPoseMessage(kf->pose);
        scene["nodes"].push_back(node);
    }

    // Add edges (factors)
    auto factors = graph.getFactors();
    for (const auto& factor : factors) {
        if (factor.connected_nodes.size() >= 2) {
            nlohmann::json edge;
            edge["from"] = std::to_string(factor.connected_nodes[0]);
            edge["to"] = std::to_string(factor.connected_nodes[1]);
            edge["type"] = static_cast<int>(factor.type);
            scene["edges"].push_back(edge);
        }
    }

    uint32_t channel_id = channel_ids_["/graph"];
    server_->broadcastMessage(channel_id, scene.dump());
}

void FoxgloveVisualizer::visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) {
    if (!is_connected_)
        return;

    // Add pose
    addPose(keyframe->pose, "keyframe_" + std::to_string(keyframe->id));

    // Add data
    if (keyframe->hasPointCloud()) {
        addPointCloud(keyframe->getPointCloud(), keyframe->pose);
    } else if (keyframe->hasImage()) {
        addImage(keyframe->getImage(), "keyframe_" + std::to_string(keyframe->id));
    }
}

nlohmann::json FoxgloveVisualizer::createPoseMessage(const core::types::Pose& pose,
                                                     const std::string& frame_id) {
    nlohmann::json msg;
    msg["timestamp"] = current_timestamp_;
    msg["frame_id"] = frame_id;
    msg["position"] = {
        {"x", pose.position.x()}, {"y", pose.position.y()}, {"z", pose.position.z()}};
    msg["orientation"] = {{"w", pose.orientation.w()},
                          {"x", pose.orientation.x()},
                          {"y", pose.orientation.y()},
                          {"z", pose.orientation.z()}};
    return msg;
}

nlohmann::json FoxgloveVisualizer::createPointCloudMessage(const core::types::PointCloud& cloud,
                                                           const core::types::Pose& pose) {
    nlohmann::json msg;
    msg["timestamp"] = current_timestamp_;
    msg["frame_id"] = "map";
    msg["pose"] = createPoseMessage(pose);

    // Pack points and colors
    std::vector<float> points;
    std::vector<uint8_t> colors;
    points.reserve(cloud.points.size() * 3);
    colors.reserve(cloud.colors.size() * 3);

    for (const auto& p : cloud.points) {
        points.push_back(static_cast<float>(p.x()));
        points.push_back(static_cast<float>(p.y()));
        points.push_back(static_cast<float>(p.z()));
    }

    for (const auto& c : cloud.colors) {
        colors.push_back(static_cast<uint8_t>(c.x() * 255));
        colors.push_back(static_cast<uint8_t>(c.y() * 255));
        colors.push_back(static_cast<uint8_t>(c.z() * 255));
    }

    msg["points"] = points;
    if (!colors.empty()) {
        msg["colors"] = colors;
    }

    return msg;
}

nlohmann::json FoxgloveVisualizer::createImageMessage(const cv::Mat& image) {
    nlohmann::json msg;
    msg["timestamp"] = current_timestamp_;
    msg["frame_id"] = "camera";
    msg["encoding"] = image.channels() == 1 ? "mono8" : "rgb8";
    msg["width"] = image.cols;
    msg["height"] = image.rows;
    msg["step"] = image.step;

    std::vector<uint8_t> data(image.data, image.data + image.total() * image.elemSize());
    msg["data"] = data;

    return msg;
}

void FoxgloveVisualizer::clear() {
    if (!is_connected_)
        return;
    // Foxglove doesn't have a direct clear mechanism
    // We could send empty updates to clear existing visualizations
}

void FoxgloveVisualizer::update() {
    if (!is_connected_)
        return;
    // Foxglove server sends messages immediately, no explicit update needed
}

void FoxgloveVisualizer::setTimestamp(double timestamp) {
    current_timestamp_ = timestamp;
}

bool FoxgloveVisualizer::isConnected() const {
    return is_connected_ && server_ && server_->numConnections() > 0;
}

void FoxgloveVisualizer::disconnect() {
    if (server_) {
        server_->stop();
    }
    is_connected_ = false;
}

void FoxgloveVisualizer::visualizeMap(const core::storage::MapStore& map_store) {
    if (!is_connected_)
        return;

    // Visualize all keyframes
    auto keyframes = map_store.getAllKeyFrames();
    for (const auto& kf : keyframes) {
        // Add pose with keyframe ID as label
        addPose(kf->pose, "keyframe_" + std::to_string(kf->id));

        // Add point cloud if available
        if (kf->hasPointCloud()) {
            addPointCloud(kf->getPointCloud(), kf->pose);
        }
    }

    // Visualize the factor graph structure
    visualizeFactorGraph(map_store.getFactorGraph());
}

void FoxgloveVisualizer::visualizeMapTimeline(const core::storage::MapStore& map_store,
                                              double current_time) {
    if (!is_connected_)
        return;

    // Clear previous visualization
    clear();

    // Get keyframes up to current_time
    auto keyframes = map_store.getAllKeyFrames();
    for (const auto& kf : keyframes) {
        if (kf->pose.timestamp <= current_time) {
            visualizeKeyFrame(kf);
        }
    }

    // Update timestamp for visualization
    setTimestamp(current_time);
}

}  // namespace viz
