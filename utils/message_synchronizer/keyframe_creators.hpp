#pragma once

namespace utils {

class KeyframeCreator
{
public:
    KeyframeCreator(double time_threshold, double distance_threshold)
        : time_threshold_(time_threshold), distance_threshold_(distance_threshold) {}

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

private:
    double time_threshold_;
    double distance_threshold_;
};
}  // namespace utils