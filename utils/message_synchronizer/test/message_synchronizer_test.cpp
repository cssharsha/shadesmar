#include "message_synchronizer/message_synchronizer.hpp"
#include <gtest/gtest.h>

class MessageSynchronizerTest : public ::testing::Test {
protected:
    struct TestMessage {
        double data;
    };

    static core::types::Pose makePose(double x, double y, double z) {
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(x, y, z);
        pose.orientation = Eigen::Quaterniond::Identity();
        return pose;
    }
};

TEST_F(MessageSynchronizerTest, PoseOnlyCallback) {
    bool callback_called = false;
    auto pose = makePose(1.0, 0.0, 0.0);

    utils::MessageSynchronizer<> synchronizer(
        [&callback_called, &pose](const core::types::Pose& p) {
            callback_called = true;
            EXPECT_EQ(p.position, pose.position);
        });

    synchronizer.addPoseMessage(pose, 1.0);
    EXPECT_TRUE(callback_called);
}

TEST_F(MessageSynchronizerTest, FullSynchronization) {
    bool full_callback_called = false;
    bool pose_callback_called = false;
    auto pose = makePose(1.0, 0.0, 0.0);
    TestMessage msg{42.0};

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&full_callback_called, &pose, &msg](const core::types::Pose& p, const TestMessage& m) {
            full_callback_called = true;
            EXPECT_EQ(p.position, pose.position);
            EXPECT_EQ(m.data, msg.data);
        },
        [&pose_callback_called, &pose](const core::types::Pose& p) {
            pose_callback_called = true;
            EXPECT_EQ(p.position, pose.position);
        });

    // Add pose first
    synchronizer.addPoseMessage(pose, 1.0);
    EXPECT_TRUE(pose_callback_called);
    EXPECT_FALSE(full_callback_called);

    // Add message
    synchronizer.addMessage(msg, 1.0);
    EXPECT_TRUE(full_callback_called);
}

TEST_F(MessageSynchronizerTest, TimeThreshold) {
    bool callback_called = false;
    auto pose1 = makePose(1.0, 0.0, 0.0);
    auto pose2 = makePose(2.0, 0.0, 0.0);
    TestMessage msg{42.0};

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&callback_called](const core::types::Pose& p, const TestMessage& m) {
            callback_called = true;
        },
        nullptr, 0.5, 0.1);  // 0.1s time threshold

    synchronizer.addPoseMessage(pose1, 1.0);
    synchronizer.addMessage(msg, 1.2);  // Outside time threshold
    EXPECT_FALSE(callback_called);

    synchronizer.addPoseMessage(pose2, 1.2);
    EXPECT_TRUE(callback_called);
}

TEST_F(MessageSynchronizerTest, DistanceThreshold) {
    int callback_count = 0;
    auto pose1 = makePose(0.0, 0.0, 0.0);
    auto pose2 = makePose(0.1, 0.0, 0.0);  // Small movement
    auto pose3 = makePose(1.0, 0.0, 0.0);  // Large movement

    utils::MessageSynchronizer<> synchronizer(
        [&callback_count](const core::types::Pose& p) { callback_count++; },
        0.5);  // 0.5m distance threshold

    synchronizer.addPoseMessage(pose1, 1.0);
    EXPECT_EQ(callback_count, 1);

    synchronizer.addPoseMessage(pose2, 1.1);  // Should not trigger (small movement)
    EXPECT_EQ(callback_count, 1);

    synchronizer.addPoseMessage(pose3, 1.2);  // Should trigger (large movement)
    EXPECT_EQ(callback_count, 2);
}

TEST_F(MessageSynchronizerTest, MultipleMessages) {
    bool callback_called = false;
    auto pose = makePose(1.0, 0.0, 0.0);

    struct TestMessage1 {
        double data;
    };

    struct TestMessage2 {
        int count;
    };

    TestMessage1 msg1{42.0};
    TestMessage2 msg2{43};

    utils::MessageSynchronizer<TestMessage1, TestMessage2> synchronizer(
        [&callback_called](const core::types::Pose& p, const TestMessage1& m1,
                           const TestMessage2& m2) {
            callback_called = true;
            EXPECT_EQ(m1.data, 42.0);
            EXPECT_EQ(m2.count, 43);
        },
        nullptr);

    synchronizer.addPoseMessage(pose, 1.0);
    synchronizer.addMessage(msg1, 1.0);
    EXPECT_FALSE(callback_called);

    synchronizer.addMessage(msg2, 1.0);
    EXPECT_TRUE(callback_called);
}

TEST_F(MessageSynchronizerTest, Cleanup) {
    bool callback_called = false;
    auto pose = makePose(1.0, 0.0, 0.0);
    TestMessage msg{42.0};

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&callback_called](const core::types::Pose& p, const TestMessage& m) {
            callback_called = true;
        },
        nullptr);

    synchronizer.addPoseMessage(pose, 1.0);
    synchronizer.addMessage(msg, 0.5);  // Old message
    synchronizer.cleanup(0.8);          // Clean up messages older than 0.8

    TestMessage new_msg{43.0};
    synchronizer.addMessage(new_msg, 1.0);
    EXPECT_TRUE(callback_called);
}

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

// Additional test cases using the mock
TEST_F(MessageSynchronizerTest, DifferentFrequencies) {
    MessagePublisherMock publisher;
    int callback_count = 0;

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&callback_count](const core::types::Pose& p, const TestMessage& m) { callback_count++; },
        nullptr,
        0.5,  // distance threshold
        0.1   // time threshold
    );

    // Configure different publishing frequencies
    MessagePublisherMock::PublishConfig pose_config{
        .frequency = 10.0,  // 10 Hz
        .time_offset = 0.0,
        .time_noise = 0.001  // 1ms noise
    };

    MessagePublisherMock::PublishConfig msg_config{
        .frequency = 5.0,  // 5 Hz
        .time_offset = 0.0,
        .time_noise = 0.001  // 1ms noise
    };

    // Generate 1 second of messages
    auto poses = publisher.generateMessages(makePose(1.0, 0.0, 0.0), pose_config, 1.0);
    auto messages = publisher.generateMessages(TestMessage{42.0}, msg_config, 1.0);

    // Process messages in time order
    size_t pose_idx = 0;
    size_t msg_idx = 0;

    while (pose_idx < poses.size() || msg_idx < messages.size()) {
        if (msg_idx >= messages.size() ||
            (pose_idx < poses.size() && poses[pose_idx].second < messages[msg_idx].second)) {
            synchronizer.addPoseMessage(poses[pose_idx].first, poses[pose_idx].second);
            pose_idx++;
        } else {
            synchronizer.addMessage(messages[msg_idx].first, messages[msg_idx].second);
            msg_idx++;
        }
    }

    // We expect approximately 5 callbacks (limited by the slower message rate)
    EXPECT_GE(callback_count, 4);
    EXPECT_LE(callback_count, 6);
}

TEST_F(MessageSynchronizerTest, DelayedMessages) {
    MessagePublisherMock publisher;
    std::vector<double> callback_timestamps;

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&callback_timestamps](const core::types::Pose& p, const TestMessage& m) {
            callback_timestamps.push_back(p.timestamp);
        },
        nullptr,
        0.5,  // distance threshold
        0.1   // time threshold
    );

    // Configure messages with delay
    MessagePublisherMock::PublishConfig pose_config{
        .frequency = 10.0, .time_offset = 0.0, .time_noise = 0.0};

    MessagePublisherMock::PublishConfig msg_config{.frequency = 10.0,
                                                   .time_offset = 0.05,  // 50ms delay
                                                   .time_noise = 0.0};

    auto poses = publisher.generateMessages(makePose(1.0, 0.0, 0.0), pose_config, 1.0);
    auto messages = publisher.generateMessages(TestMessage{42.0}, msg_config, 1.0);

    // Process all poses first, then all messages (simulating delayed arrival)
    for (const auto& [pose, timestamp] : poses) {
        synchronizer.addPoseMessage(pose, timestamp);
    }
    for (const auto& [msg, timestamp] : messages) {
        synchronizer.addMessage(msg, timestamp);
    }

    // Check that callbacks were triggered with correct timing
    ASSERT_FALSE(callback_timestamps.empty());
    for (size_t i = 1; i < callback_timestamps.size(); i++) {
        double dt = callback_timestamps[i] - callback_timestamps[i - 1];
        EXPECT_NEAR(dt, 0.1, 0.02);  // Expect ~10Hz callbacks
    }
}

TEST_F(MessageSynchronizerTest, OutOfOrderMessages) {
    MessagePublisherMock publisher;
    std::vector<double> synchronized_times;

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&synchronized_times](const core::types::Pose& p, const TestMessage& m) {
            synchronized_times.push_back(p.timestamp);
        },
        nullptr,
        0.5,  // distance threshold
        0.2   // time threshold
    );

    // Generate messages
    auto poses = publisher.generateMessages(makePose(1.0, 0.0, 0.0), {10.0, 0.0, 0.0}, 1.0);
    auto messages = publisher.generateMessages(TestMessage{42.0}, {10.0, 0.0, 0.0}, 1.0);

    // Randomly shuffle message order
    auto shuffled_poses = poses;
    auto shuffled_messages = messages;
    std::random_shuffle(shuffled_poses.begin(), shuffled_poses.end());
    std::random_shuffle(shuffled_messages.begin(), shuffled_messages.end());

    // Process messages in shuffled order
    for (const auto& [pose, timestamp] : shuffled_poses) {
        synchronizer.addPoseMessage(pose, timestamp);
    }
    for (const auto& [msg, timestamp] : shuffled_messages) {
        synchronizer.addMessage(msg, timestamp);
    }

    // Verify that synchronized timestamps are in order
    ASSERT_FALSE(synchronized_times.empty());
    for (size_t i = 1; i < synchronized_times.size(); i++) {
        EXPECT_GT(synchronized_times[i], synchronized_times[i - 1]);
    }
}

TEST_F(MessageSynchronizerTest, MessageDropout) {
    MessagePublisherMock publisher;
    int callback_count = 0;

    utils::MessageSynchronizer<TestMessage> synchronizer(
        [&callback_count](const core::types::Pose& p, const TestMessage& m) { callback_count++; },
        [](const core::types::Pose& p) {
            // Pose-only callback
        },
        0.5,  // distance threshold
        0.1   // time threshold
    );

    // Generate messages with some dropouts
    auto poses = publisher.generateMessages(makePose(1.0, 0.0, 0.0), {10.0, 0.0, 0.0}, 1.0);
    auto messages = publisher.generateMessages(TestMessage{42.0}, {10.0, 0.0, 0.0}, 1.0);

    // Simulate 20% message dropout
    auto dropout = [](const auto& messages, double dropout_rate) {
        auto result = messages;
        result.erase(std::remove_if(result.begin(), result.end(),
                                    [dropout_rate](const auto&) {
                                        return (rand() / (double)RAND_MAX) < dropout_rate;
                                    }),
                     result.end());
        return result;
    };

    auto poses_with_dropout = dropout(poses, 0.2);
    auto messages_with_dropout = dropout(messages, 0.2);

    // Process messages
    size_t pose_idx = 0;
    size_t msg_idx = 0;

    while (pose_idx < poses_with_dropout.size() || msg_idx < messages_with_dropout.size()) {
        if (msg_idx >= messages_with_dropout.size() ||
            (pose_idx < poses_with_dropout.size() &&
             poses_with_dropout[pose_idx].second < messages_with_dropout[msg_idx].second)) {
            synchronizer.addPoseMessage(poses_with_dropout[pose_idx].first,
                                        poses_with_dropout[pose_idx].second);
            pose_idx++;
        } else {
            synchronizer.addMessage(messages_with_dropout[msg_idx].first,
                                    messages_with_dropout[msg_idx].second);
            msg_idx++;
        }
    }

    // Expect fewer callbacks due to dropouts
    EXPECT_LT(callback_count, poses.size());
}
