#pragma once

#include <deque>
#include <functional>
#include <iomanip>
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

    template <typename T, typename First, typename... Rest>
    std::optional<std::pair<double, T>> getMessage(double timestamp) {
        if constexpr (std::is_same_v<T, First>) {
            std::optional<std::pair<double, T>> found_msg = std::nullopt;
            auto queue = std::get<std::deque<std::pair<double, First>>>(message_queues_);
            while (!queue.empty()) {
                auto front = queue.front();
                auto diff = std::abs(queue.front().first - timestamp);

                if (diff < 0.5) {
                    found_msg = std::move(front);
                    queue.pop_front();
                    break;
                } else if (queue.front().first < timestamp) {
                    queue.pop_front();
                } else {
                    break;
                }
            }
            if (found_msg.has_value()) {
                LOG(INFO) << "Found a message!!";
            }
            return found_msg;
        } else if constexpr (sizeof...(Rest) > 0) {
            return getMessage<T, Rest...>(timestamp);
        } else {
            return std::nullopt;
        }
    }

    // Try to call callback with given pose and timestamp
    bool tryCallback(const core::types::Pose& pose, double timestamp, bool require_all = false) {
        // Get the messages tuple
        auto img_msg = getMessage<core::types::Image, Messages...>(timestamp);
        auto cam_info_msg = getMessage<core::types::CameraInfo, Messages...>(timestamp);

        LOG(INFO) << std::fixed << "Synchronized messages found, passing to callback at "
                  << timestamp << "image: " << img_msg.has_value()
                  << "cam info: " << cam_info_msg.has_value();

        std::optional<core::types::Image> msg1 =
            (img_msg.has_value()) ? std::make_optional(img_msg.value().second) : std::nullopt;
        std::optional<core::types::CameraInfo> msg2 =
            (cam_info_msg.has_value()) ? std::make_optional(cam_info_msg.value().second)
                                       : std::nullopt;

        // Apply the callback with the rest of the tuple (skipping the success flag)
        keyframe_callback_(pose, msg1, msg2);
        cleanup(timestamp);
        return img_msg.has_value() || cam_info_msg.has_value();
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
