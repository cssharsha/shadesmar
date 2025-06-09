#pragma once

#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include "core/types/pose.hpp"

#include "logging/logging.hpp"

namespace utils {

template <typename... Messages>
class MessageSynchronizer {
public:
    using KeyframeCallback = std::function<std::shared_ptr<core::types::KeyFrame>(
        const core::types::Pose&, const std::optional<Messages>&...)>;

    struct SyncConfig {
        double time_threshold = 1.0;   // Time threshold for processing
        double sync_tolerance = 0.1;   // Time tolerance for message synchronization
        size_t max_queue_size = 1000;  // Maximum queue size before dropping messages
    };

    MessageSynchronizer(KeyframeCallback callback, double time_threshold = 1.0)
        : keyframe_callback_(std::move(callback)) {
        config_.time_threshold = time_threshold;
    }

    MessageSynchronizer(KeyframeCallback callback, const SyncConfig& config)
        : keyframe_callback_(std::move(callback)), config_(config) {}

    // Add pose message - this drives the synchronization process
    void addPoseMessage(const core::types::Pose& pose, double timestamp) {
        pose_messages_processed_++;
        LOG(INFO) << "Adding pose message #" << pose_messages_processed_ << " at timestamp "
                  << std::fixed << std::setprecision(3) << timestamp;

        // Add to pose queue in sorted order
        insertSorted(pose_queue_, {timestamp, pose});

        // Limit queue size
        if (pose_queue_.size() > config_.max_queue_size) {
            pose_queue_.pop_front();
            LOG(WARNING) << "Pose queue size exceeded, dropping oldest message";
        }

        // Process pending messages
        processMessages();
    }

    // Add other message types
    template <typename T>
    void addMessage(const T& msg, double timestamp) {
        static_assert((std::is_same_v<T, Messages> || ...), "Unsupported message type");

        auto& queue = getQueue<T>();
        insertSorted(queue, {timestamp, msg});

        // Limit queue size
        if (queue.size() > config_.max_queue_size) {
            queue.pop_front();
            LOG(WARNING) << "Message queue size exceeded for type " << typeid(T).name();
        }

        LOG(INFO) << "Added message of type " << typeid(T).name() << " at timestamp " << std::fixed
                  << std::setprecision(3) << timestamp;
    }

    // Configuration
    void setDistanceThreshold(double threshold) {
        config_.distance_threshold = threshold;
    }
    void setTimeThreshold(double threshold) {
        config_.time_threshold = threshold;
    }
    void setConfig(const SyncConfig& config) {
        config_ = config;
    }

    // Statistics
    size_t getPoseMessagesProcessed() const {
        return pose_messages_processed_;
    }
    size_t getPoseQueueSize() const {
        return pose_queue_.size();
    }

    template <typename T>
    size_t getQueueSize() const {
        return getQueue<T>().size();
    }

private:
    using TimestampedPose = std::pair<double, core::types::Pose>;
    template <typename T>
    using TimestampedMessage = std::pair<double, T>;

    // Message storage
    std::deque<TimestampedPose> pose_queue_;
    std::tuple<std::deque<TimestampedMessage<Messages>>...> message_queues_;

    // Configuration and state
    KeyframeCallback keyframe_callback_;
    SyncConfig config_;
    size_t pose_messages_processed_ = 0;
    std::shared_ptr<core::types::KeyFrame> last_keyframe_ = nullptr;

    // Get queue for specific message type
    template <typename T>
    auto& getQueue() {
        return std::get<std::deque<TimestampedMessage<T>>>(message_queues_);
    }

    template <typename T>
    const auto& getQueue() const {
        return std::get<std::deque<TimestampedMessage<T>>>(message_queues_);
    }

    // Insert message in sorted order (by timestamp)
    template <typename T>
    void insertSorted(std::deque<T>& queue, const T& item) {
        auto it = std::upper_bound(queue.begin(), queue.end(), item,
                                   [](const T& a, const T& b) { return a.first < b.first; });
        queue.insert(it, item);
    }

    // Find synchronized message within tolerance
    template <typename T>
    std::optional<T> findSynchronizedMessage(double target_timestamp) {
        auto& queue = getQueue<T>();

        if constexpr (std::is_same_v<T, core::types::ImuData>) {
            LOG(INFO) << "IMU sync: Looking for timestamp " << std::fixed << std::setprecision(3)
                      << target_timestamp << ", queue size: " << queue.size()
                      << ", tolerance: " << config_.sync_tolerance << "s";

            if (!queue.empty()) {
                LOG(INFO) << "   Queue range: " << queue.front().first << " → "
                          << queue.back().first;
            }
        }

        // Find closest message within tolerance
        auto best_it = queue.end();
        double best_diff = std::numeric_limits<double>::max();

        for (auto it = queue.begin(); it != queue.end(); ++it) {
            double diff = std::abs(it->first - target_timestamp);
            if (diff <= config_.sync_tolerance && diff < best_diff) {
                best_diff = diff;
                best_it = it;
            }
        }

        if (best_it != queue.end()) {
            T message = best_it->second;
            queue.erase(best_it);
            LOG(INFO) << "Found synchronized " << typeid(T).name()
                      << " message (time diff: " << std::fixed << std::setprecision(3) << best_diff
                      << "s)";
            return message;
        }

        if constexpr (std::is_same_v<T, core::types::ImuData>) {
            LOG(WARNING) << "No IMU data found within tolerance for timestamp " << target_timestamp;
            if (!queue.empty()) {
                LOG(WARNING) << "   Closest: " << queue.front().first
                             << " (diff: " << std::abs(queue.front().first - target_timestamp)
                             << "s)";
            }
        }

        return std::nullopt;
    }

    // Create tuple of synchronized messages
    std::tuple<std::optional<Messages>...> getSynchronizedMessages(double timestamp) {
        return std::make_tuple(findSynchronizedMessage<Messages>(timestamp)...);
    }

    // Process a single pose with synchronized messages
    void processPose(const core::types::Pose& pose, double timestamp) {
        // Always process poses - let the callback (GraphAdapter/KeyframeManager) decide on keyframe
        // creation This removes the redundant distance-based filtering that was conflicting with
        // KeyframeManager

        // Get synchronized messages
        auto synchronized_messages = getSynchronizedMessages(timestamp);

        LOG(INFO) << "Processing pose at timestamp " << std::fixed << std::setprecision(3)
                  << timestamp;

        // Call callback with synchronized messages - callback will decide if keyframe should be
        // created
        auto keyframe = std::apply([&](auto... msgs) { return keyframe_callback_(pose, msgs...); },
                                   synchronized_messages);

        if (keyframe) {
            last_keyframe_ = keyframe;
            LOG(INFO) << "Keyframe created with ID: " << keyframe->id;
        } else {
            LOG(INFO) << "No keyframe created (decision made by KeyframeManager)";
        }
    }

    // Process all messages that are ready
    void processMessages() {
        if (pose_queue_.empty()) {
            return;
        }

        // Only process messages if we have enough time buffer or enough messages
        double oldest = pose_queue_.front().first;
        double newest = pose_queue_.back().first;
        double time_span = newest - oldest;

        if (time_span < config_.time_threshold && pose_queue_.size() < 10) {
            LOG(INFO) << "Waiting for more messages (time span: " << std::fixed
                      << std::setprecision(3) << time_span << "s)";
            return;
        }

        // Process poses that have enough time for synchronization
        double processing_cutoff = newest - config_.time_threshold;

        size_t processed_count = 0;
        auto it = pose_queue_.begin();
        while (it != pose_queue_.end() && it->first <= processing_cutoff) {
            processPose(it->second, it->first);
            it = pose_queue_.erase(it);
            processed_count++;
        }

        if (processed_count > 0) {
            LOG(INFO) << "Processed " << processed_count << " pose messages";
            cleanupOldMessages(processing_cutoff);
        }
    }

    // Clean up old messages from all queues
    void cleanupOldMessages(double cutoff_timestamp) {
        double cleanup_threshold = cutoff_timestamp - config_.time_threshold;

        // Clean up message queues
        (cleanupQueue(getQueue<Messages>(), cleanup_threshold), ...);

        LOG(INFO) << "Cleaned up messages older than " << std::fixed << std::setprecision(3)
                  << cleanup_threshold;
    }

    // Clean up specific queue
    template <typename T>
    void cleanupQueue(std::deque<T>& queue, double threshold) {
        size_t initial_size = queue.size();

        while (!queue.empty() && queue.front().first < threshold) {
            queue.pop_front();
        }

        size_t removed = initial_size - queue.size();
        if (removed > 0) {
            LOG(INFO) << "Cleaned up " << removed << " old messages of type "
                      << typeid(typename T::second_type).name();
        }
    }
};

}  // namespace utils
