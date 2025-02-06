#pragma once

#include <deque>

namespace utils {

template <typename T>
class MessageQueue {
private:
    std::deque<T> queue_;

public:
    // Default constructor
    MessageQueue() = default;

    // Destructor
    ~MessageQueue() = default;

    // Copy constructor
    MessageQueue(const MessageQueue& other) : queue_(other.queue_) {}

    // Copy assignment operator
    MessageQueue& operator=(const MessageQueue& other) {
        if (this != &other) {
            queue_ = other.queue_;
        }
        return *this;
    }

    // Move constructor
    MessageQueue(MessageQueue&& other) noexcept : queue_(std::move(other.queue_)) {}

    // Move assignment operator
    MessageQueue& operator=(MessageQueue&& other) noexcept {
        if (this != &other) {
            queue_ = std::move(other.queue_);
        }
        return *this;
    }

    // Check if queue has messages at the given timestamp and return matching message
    std::tuple<bool, std::optional<typename T::second_type>> tryGetMessages(
        double timestamp, bool require_all = true) const {
        LOG(INFO) << "Checking messages at timestamp " << std::fixed << timestamp
                  << " (require_all=" << require_all << ")";

        std::tuple<bool, std::optional<typename T::second_type>> result;
        std::get<0>(result) = false;  // success flag
        std::get<1>(result) = std::nullopt;

        LOG(INFO) << "Checking queue of type " << typeid(T).name() << " (size=" << queue_.size()
                  << ")";

        if (!queue_.empty()) {
            // Make temporary copy to iterate through
            auto temp_queue = queue_;
            double smallest_diff = std::numeric_limits<double>::max();
            std::optional<typename T::second_type> closest_message = std::nullopt;

            // Print timestamp range for debugging
            LOG(INFO) << "  Queue timestamp range: [" << std::fixed << temp_queue.front().first
                      << ", " << temp_queue.back().first << "]";

            // Check each message
            while (!temp_queue.empty()) {
                const auto& msg = temp_queue.front();
                double diff = std::abs(msg.first - timestamp);

                if (diff < smallest_diff) {
                    smallest_diff = diff;
                    closest_message = msg.second;
                }
                temp_queue.pop_front();
            }

            LOG(INFO) << "  Smallest timestamp diff found: " << smallest_diff;
            if (smallest_diff < 0.5 && closest_message.has_value()) {
                std::get<0>(result) = true;
                std::get<1>(result) = closest_message;
                LOG(INFO) << "  Found matching message";
            } else {
                LOG(INFO) << "  No matching message (timestamp diff too large)";
            }
        } else {
            LOG(INFO) << "  Queue is empty";
        }

        LOG(INFO) << "Message check result: " << (std::get<0>(result) ? "success" : "failure");
        return result;
    }

    std::deque<T>& queue() {
        return queue_;
    }
};

}  // namespace utils
