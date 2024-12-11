load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary", "cc_test")

cc_library(
    name = "ros_conversions",
    srcs = ["src/conversions.cpp"],
    hdrs = ["include/ros/conversions.hpp"],
    includes = ["include"],
    deps = [
        "//core:core",
        "@ros_core//:rcl_base",
        "@ros_core//:core",
        "@ros_core//:cv_bridge",
        "@ros_core//:geometry_msgs",
        "@ros_core//:sensor_msgs",
        "@ros_core//:nav_msgs",
        "@ros_core//:message_filters",
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "rosbag_reader",
    srcs = ["src/rosbag_reader.cpp"],
    hdrs = ["include/ros/rosbag_reader.hpp"],
    includes = ["include"],
    deps = [
        ":ros_conversions",
        "//core:core",
        "@ros_core//:rosbag",
        "@ros_core//:message_filters",
    ],
    visibility = ["//visibility:public"],
)

# Test targets
cc_test(
    name = "conversions_test",
    srcs = ["test/conversions_test.cpp"],
    deps = [
        ":ros_conversions",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "rosbag_reader_test",
    srcs = ["test/rosbag_reader_test.cpp"],
    deps = [
        ":rosbag_reader",
        "@com_google_googletest//:gtest_main",
        "@ros_core//:rosbag",
        "@ros_core//:nav_msgs",
        "@ros_core//:sensor_msgs",
        "@ros_core//:geometry_msgs",
    ],
    # Add data dependency for test bag files if needed
    # data = [
    #     "test/data/test.bag",
    # ],
)

# Convenience test suite that runs all ROS tests
test_suite(
    name = "ros_tests",
    tests = [
        ":conversions_test",
        ":rosbag_reader_test",
    ],
)