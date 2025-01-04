#pragma once

class MessagePublisherMock {
public:
    MessagePublisherMock(double start_time = 0.0) : current_time_(start_time) {}

    struct PublishConfig {
        double frequency;    // Hz
        double time_offset;  // Initial offset in seconds
        double time_noise;   // Random noise in timing (+/- seconds)
    };

    template <typename T>
    std::vector<std::pair<T, double>> generateMessages(const T& base_msg,
                                                       const PublishConfig& config,
                                                       double duration) {
        std::vector<std::pair<T, double>> messages;
        double period = 1.0 / config.frequency;
        double next_time = current_time_ + config.time_offset;
        double end_time = current_time_ + duration;

        while (next_time < end_time) {
            // Add random timing noise
            double noise = (2.0 * rand() / RAND_MAX - 1.0) * config.time_noise;
            double timestamp = next_time + noise;
            messages.emplace_back(base_msg, timestamp);
            next_time += period;
        }

        return messages;
    }

    void advanceTime(double seconds) {
        current_time_ += seconds;
    }

private:
    double current_time_;
};
