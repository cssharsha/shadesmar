#pragma once

#include <deque>
#include <functional>
#include <optional>
#include <tuple>
#include <utility>
#include "core/types/pose.hpp"

#include "logging/logging.hpp"

namespace utils {

template <typename... Messages>
class MessageSynchronizer {
public:
    using KeyframeCallback =
        std::function<void(const core::types::Pose&, const std::optional<Messages>&...)>;

    MessageSynchronizer(KeyframeCallback callback, double distance_threshold = 0.5,
                        double time_threshold = 1.0)
        : keyframe_callback_(std::move(callback)),
          distance_threshold_(distance_threshold),
          time_threshold_(time_threshold) {}

    // Add pose message
    void addPoseMessage(const core::types::Pose& msg, double timestamp) {
        // LOG(DEBUG) << "Adding pose message at timestamp " << timestamp;

        if (!last_callback_pose_) {
            // LOG(DEBUG) << "First pose message received";
        } else {
            double distance = (msg.position - last_callback_pose_->position).norm();
            // LOG(DEBUG) << "Distance from last pose: " << distance;
        }

        if (!last_callback_pose_ ||
            (msg.position - last_callback_pose_->position).norm() >= distance_threshold_) {
            // LOG(DEBUG) << "Distance threshold exceeded or first message, attempting callback";

            if (tryCallback(msg, timestamp)) {
                // LOG(DEBUG) << "Callback successful";
                last_callback_pose_ = msg;
            } else {
                // LOG(DEBUG) << "Callback unsuccessful, processing queue";
                while (!pose_queue_.empty() &&
                       (timestamp - pose_queue_.front().first) > time_threshold_) {
                    // LOG(DEBUG) << "Processing queued pose from timestamp "
                    //            << pose_queue_.front().first;

                    if (tryCallback(pose_queue_.front().second, pose_queue_.front().first)) {
                        // LOG(DEBUG) << "Queued pose callback successful";
                        last_callback_pose_ = pose_queue_.front().second;
                        last_callback_timestamp_ = pose_queue_.front().first;
                    } else if (keyframe_callback_) {
                        // LOG(DEBUG) << "Falling back to pose-only callback";
                        keyframe_callback_(pose_queue_.front().second, std::nullopt, std::nullopt);
                        last_callback_pose_ = pose_queue_.front().second;
                        last_callback_timestamp_ = pose_queue_.front().first;
                    }
                    pose_queue_.pop_front();
                }
                // LOG(DEBUG) << "Adding new pose to queue";
                addToDeque(pose_queue_, msg, timestamp);
            }
        } else {
            // LOG(DEBUG) << "Distance threshold not exceeded, skipping";
        }
    }

    // Get distance from last pose to given pose
    std::optional<double> getDistanceFromLastPose(const core::types::Pose& pose) const {
        if (pose_queue_.empty()) {
            return std::nullopt;
        }
        return (pose_queue_.back().second.position - pose.position).norm();
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
        LOG(INFO) << "Cleaning up messages older than " << timestamp;

        int pose_count_before = pose_queue_.size();
        LOG(INFO) << "Pose queue size before cleanup: " << pose_count_before;
        cleanupDeque(pose_queue_, timestamp);
        LOG(INFO) << "Removed " << (pose_count_before - pose_queue_.size()) << " pose messages";

        if constexpr (sizeof...(Messages) > 0) {
            (cleanupDeque(std::get<std::deque<std::pair<double, Messages>>>(message_queues_),
                          timestamp),
             ...);
        }
    }

    void setDistanceThreshold(double threshold) {
        distance_threshold_ = threshold;
    }

private:
    std::deque<std::pair<double, core::types::Pose>> pose_queue_;
    std::tuple<std::deque<std::pair<double, Messages>>...> message_queues_;
    KeyframeCallback keyframe_callback_;
    std::optional<core::types::Pose> last_callback_pose_;
    double distance_threshold_;
    std::optional<double> last_callback_timestamp_;
    double time_threshold_;

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
        if (last_callback_timestamp_ && timestamp < *last_callback_timestamp_) {
            // LOG(DEBUG) << "Skipping message older than last callback";
            return;
        }

        if constexpr (std::is_same_v<T, First>) {
            // LOG(DEBUG) << "Message type match found, adding to appropriate queue";
            addToDeque(std::get<std::deque<std::pair<double, First>>>(message_queues_), msg,
                       timestamp);
            tryCallback();
        } else if constexpr (sizeof...(Rest) > 0) {
            // LOG(DEBUG) << "Message type mismatch, trying next type";
            addToDequeByType<T, Rest...>(msg, timestamp);
        }
    }

    // Helper to cleanup deque
    template <typename T>
    void cleanupDeque(std::deque<std::pair<double, T>>& queue, double timestamp) {
        while (!queue.empty() && queue.front().first < timestamp) {
            queue.pop_front();
        }
    }

    // Try to call callback with given pose and timestamp
    bool tryCallback(const core::types::Pose& pose, double timestamp) {
        if (keyframe_callback_) {
            if (tryGetMessages(timestamp)) {
                // If we have synchronized messages, pass them as optionals
                std::apply(
                    [this, &pose](const auto&... queues) {
                        keyframe_callback_(pose, std::optional<Messages>()...);
                    },
                    message_queues_);
            } else {
                // If we don't have synchronized messages, pass nullopt
                keyframe_callback_(pose, std::nullopt, std::nullopt);
            }
            last_callback_timestamp_ = timestamp;
            cleanup(timestamp);
            return true;
        }
        return false;
    }

    // Try to call callback with queued messages
    void tryCallback() {
        if (pose_queue_.empty()) {
            // LOG(DEBUG) << "No poses in queue, skipping callback";
            return;
        }

        const auto& [pose_time, pose] = pose_queue_.back();
        // LOG(DEBUG) << "Trying callback with latest pose at timestamp " << pose_time;

        if (tryCallback(pose, pose_time)) {
            // LOG(DEBUG) << "Callback successful, removing used pose";
            last_callback_pose_ = pose;
            pose_queue_.pop_back();
        }
    }

    // Check if all queues have messages at the given timestamp
    bool tryGetMessages(double timestamp) const {
        // LOG(DEBUG) << "Checking message availability at timestamp " << timestamp;

        bool result = std::apply(
            [timestamp](const auto&... queues) {
                return ((!queues.empty() && std::abs(queues.back().first - timestamp) < 0.1) &&
                        ...);
            },
            message_queues_);

        // LOG(DEBUG) << "Message availability check result: "
        //            << (result ? "available" : "unavailable");
        return result;
    }
};

}  // namespace utils
