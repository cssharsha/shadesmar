#include "message_synchronizer/test/message_synchronizer_mock.hpp"
#include <gtest/gtest.h>
#include "message_synchronizer/message_synchronizer.hpp"

class MessageSynchronizerMockTest : public ::testing::Test {
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

TEST_F(MessageSynchronizerMockTest, DifferentFrequencies) {
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

TEST_F(MessageSynchronizerMockTest, DelayedMessages) {
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

TEST_F(MessageSynchronizerMockTest, OutOfOrderMessages) {
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

TEST_F(MessageSynchronizerMockTest, MessageDropout) {
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
