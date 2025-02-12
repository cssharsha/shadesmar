#pragma once

#include <deque>
#include <functional>
#include <iomanip>
#include <optional>
#include <tuple>
#include <utility>
#include "core/types/pose.hpp"

#include "logging/logging.hpp"

namespace utils {

template <typename... Messages>
class MessageSynchronizer {
public:
    using KeyframeCallback = std::function<std::shared_ptr<core::types::KeyFrame>(
        const core::types::Pose&, const std::optional<Messages>&...)>;

    MessageSynchronizer(KeyframeCallback callback, double distance_threshold = 0.5,
                        double time_threshold = 1.0)
        : keyframe_callback_(std::move(callback)),
          distance_threshold_(distance_threshold),
          time_threshold_(time_threshold) {}

    // Add pose message
    void addPoseMessage(const core::types::Pose& msg, double timestamp) {
        pose_messages_processed_++;

        LOG(INFO) << "Adding new pose message #" << pose_messages_processed_ << " to queue";

        addToDeque(pose_queue_, msg, timestamp);
        tryCallback();
    }

    // Add other type of message
    template <typename T>
    void addMessage(const T& msg, double timestamp) {
        // LOG(DEBUG) << "Adding message of type " << typeid(T).name() << " at timestamp "
        //            << timestamp;

        if constexpr (sizeof...(Messages) > 0) {
            addToDequeByType<T, Messages...>(msg, timestamp);
        }
    }

    // Cleanup messages older than timestamp
    void cleanup(double timestamp) {
        LOG(INFO) << std::fixed << "Cleaning up messages older than " << timestamp;

        int pose_count_before = pose_queue_.size();
        LOG(INFO) << "Pose queue size before cleanup: " << pose_count_before;
        cleanupDeque(pose_queue_, timestamp);
        LOG(INFO) << "Removed " << (pose_count_before - pose_queue_.size()) << " pose messages";

        if constexpr (sizeof...(Messages) > 0) {
            LOG(INFO) << "Cleaning up message queues, number of queues: " << sizeof...(Messages);
            try {
                (cleanupDeque(std::get<std::deque<std::pair<double, Messages>>>(message_queues_),
                              timestamp),
                 ...);
                LOG(INFO) << "Message queues cleanup completed successfully";
            } catch (const std::exception& e) {
                LOG(ERROR) << "Exception during message queues cleanup: " << e.what();
            } catch (...) {
                LOG(ERROR) << "Unknown exception during message queues cleanup";
            }
        }
    }

    void setDistanceThreshold(double threshold) {
        distance_threshold_ = threshold;
    }

    // Add getter for statistics
    size_t getPoseMessagesProcessed() const {
        return pose_messages_processed_;
    }

private:
    std::deque<std::pair<double, core::types::Pose>> pose_queue_;
    std::tuple<std::deque<std::pair<double, Messages>>...> message_queues_;
    KeyframeCallback keyframe_callback_;
    double distance_threshold_;
    double time_threshold_;
    size_t pose_messages_processed_ = 0;
    std::shared_ptr<core::types::KeyFrame> last_keyframe_ = nullptr;

    // Helper to add message to deque in sorted order
    template <typename T>
    void addToDeque(std::deque<std::pair<double, T>>& queue, const T& msg, double timestamp) {
        // LOG(DEBUG) << "Adding message to queue at timestamp " << timestamp;

        auto it = queue.begin();
        while (it != queue.end() && it->first < timestamp) {
            ++it;
        }
        queue.insert(it, std::make_pair(timestamp, msg));
        // LOG(DEBUG) << "Queue size after insertion: " << queue.size();
    }

    // Helper to add message to correct deque based on type
    template <typename T, typename First, typename... Rest>
    void addToDequeByType(const T& msg, double timestamp) {
        if constexpr (std::is_same_v<T, First>) {
            // LOG(DEBUG) << "Message type match found, adding to appropriate queue";
            addToDeque(std::get<std::deque<std::pair<double, First>>>(message_queues_), msg,
                       timestamp);
        } else if constexpr (sizeof...(Rest) > 0) {
            // LOG(DEBUG) << "Message type mismatch, trying next type";
            addToDequeByType<T, Rest...>(msg, timestamp);
        }
    }

    // Helper to cleanup deque
    template <typename T>
    void cleanupDeque(std::deque<std::pair<double, T>>& queue, double timestamp) {
        LOG(INFO) << std::fixed << "Cleaning up deque of type " << typeid(T).name();

        if (queue.empty()) {
            LOG(INFO) << "Queue is empty, nothing to clean";
            return;
        }

        LOG(INFO) << std::fixed << " - Queue size before: " << queue.size()
                  << "\n - Front timestamp: " << queue.front().first
                  << "\n - Back timestamp: " << queue.back().first
                  << "\n - Cleanup timestamp: " << timestamp;

        size_t initial_size = queue.size();
        while (!queue.empty() && queue.front().first < timestamp + 1e-6) {
            LOG(INFO) << std::fixed << "Popping message at timestamp " << queue.front().first;
            queue.pop_front();
        }
        LOG(INFO) << "Cleaned up " << (initial_size - queue.size()) << " messages";
    }

    // Check if all queues have messages at the given timestamp and return matching messages
    std::tuple<bool, std::optional<Messages>...> tryGetMessages(double timestamp,
                                                                bool require_all = true) const {
        LOG(INFO) << "Checking messages at timestamp " << std::fixed << timestamp
                  << " (require_all=" << require_all << ")";

        std::tuple<bool, std::optional<Messages>...> result;
        std::get<0>(result) = false;  // success flag

        bool has_any_message = false;
        auto check_and_get_messages = [&](const auto&... queues) {
            bool all_present = true;
            (
                (void)[&] {
                    using QueueType = std::decay_t<decltype(queues)>;
                    using MessageType = typename QueueType::value_type::second_type;

                    LOG(INFO) << "Checking queue of type " << typeid(MessageType).name()
                              << " (size=" << queues.size() << ")";

                    if (!queues.empty()) {
                        // Print timestamp range of messages in queue
                        LOG(INFO) << "  Queue timestamp range: [" << std::fixed
                                  << queues.front().first << ", " << queues.back().first << "]";
                        double diff = std::abs(queues.back().first - timestamp);
                        LOG(INFO) << "  Latest message timestamp diff: " << diff;
                        if (diff < 0.5) {
                            has_any_message = true;
                            std::get<std::optional<MessageType>>(result) = queues.back().second;
                            LOG(INFO) << "  Found matching message";
                        } else {
                            all_present = false;
                            LOG(INFO) << "  No matching message (timestamp diff too large)";
                        }
                    } else {
                        all_present = false;
                        LOG(INFO) << "  Queue is empty";
                    }
                }(),
                ...);

            std::get<0>(result) = require_all ? all_present : has_any_message;
        };

        std::apply(check_and_get_messages, message_queues_);
        LOG(INFO) << "Message check result: " << (std::get<0>(result) ? "success" : "failure");
        return result;
    }

    // Try to call callback with given pose and timestamp
    bool tryCallback(const core::types::Pose& pose, double timestamp, bool require_all = false) {
        // Get the messages tuple
        auto messages_tuple = tryGetMessages(timestamp, require_all);
        bool success = std::get<0>(messages_tuple);

        if (success) {
            LOG(INFO) << std::fixed << "Synchronized messages found, passing to callback at "
                      << timestamp;
            // Apply the callback with the rest of the tuple (skipping the success flag)
            std::apply(
                [&](const auto&... msgs) { last_keyframe_ = keyframe_callback_(pose, msgs...); },
                std::tuple_cat(std::make_tuple(),
                               std::tuple<std::optional<Messages>&...>(
                                   std::get<std::optional<Messages>>(messages_tuple)...)));

            LOG(INFO) << "Cleaning up messages older than " << timestamp;
            cleanup(timestamp);
            return true;
        }
        return false;
    }

    // Try to call callback with queued messages
    void tryCallback() {
        if (pose_queue_.empty()) {
            LOG(INFO) << "No poses in queue, skipping callback";
            return;
        }

        if (std::abs(pose_queue_.back().first - pose_queue_.front().first) < time_threshold_) {
            LOG(INFO) << std::fixed
                      << "Time threshold not met, waiting for multiple messages to queue: "
                      << std::abs(pose_queue_.back().first - pose_queue_.front().first)
                      << "Pose queue size: " << pose_queue_.size();
            return;
        }

        LOG(INFO) << std::fixed
                  << "Trying callback with poses in queue (size=" << pose_queue_.size()
                  << ", first=" << pose_queue_.front().first
                  << ", last=" << pose_queue_.back().first << ")";

        for (const auto& [pose_time, pose] : pose_queue_) {
            if (last_keyframe_) {
                auto distance = (last_keyframe_->pose.position - pose.position).norm();
                auto lastKeyframeHasMessages = last_keyframe_->hasColorImage() ||
                                               last_keyframe_->hasImage() ||
                                               last_keyframe_->hasPointCloud();
                if (!lastKeyframeHasMessages || distance > distance_threshold_) {
                    if (!tryCallback(pose, pose_time, false)) {
                        if (distance > distance_threshold_) {
                            last_keyframe_ = keyframe_callback_(pose, std::optional<Messages>()...);
                            cleanup(pose_time);
                            break;
                        }
                    } else {
                        break;
                    }
                }
            } else {
                last_keyframe_ = keyframe_callback_(pose, std::optional<Messages>()...);
                cleanupDeque(pose_queue_, pose_time);
                break;
            }
        }
    }
};

}  // namespace utils
