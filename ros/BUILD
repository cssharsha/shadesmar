load("@rules_cc//cc:defs.bzl", "cc_test")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_test")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_binary")

ros2_cpp_library(
    name = "ros_conversions",
    srcs = ["src/conversions.cpp"],
    hdrs = ["include/ros/conversions.hpp"],
    includes = ["include"],
    deps = [
        "//core:core",
        "@ros2_rclcpp//:rclcpp",                # ROS2 C++ client library
        "@ros2_common_interfaces//:cpp_geometry_msgs",         # Geometry messages
        "@ros2_common_interfaces//:cpp_sensor_msgs",           # Sensor messages
        "@ros2_common_interfaces//:cpp_nav_msgs",             # Navigation messages
        "@ros2_common_interfaces//:cpp_std_msgs",             # Standard messages
        "@ros2_geometry2//:cpp_tf2_msgs",             # TF messages
        "@ros2_geometry2//:tf2_ros",
        "@ros2_message_filters//:message_filters",       # Message filtering
        "@opencv//:core",
        "@opencv//:imgproc",
    ],
    visibility = ["//visibility:public"],
)

ros2_cpp_library(
    name = "rosbag_reader",
    srcs = ["src/rosbag_reader.cpp"],
    hdrs = ["include/ros/rosbag_reader.hpp"],
    includes = ["include"],
    deps = [
        "//core:core",
        "//viz:rerun_viz",
        ":ros_conversions",
        "@ros2_rosbag2//:rosbag2_cpp",          # ROS2 bag reading
        "@ros2_message_filters//:message_filters",
        "//utils:stf",
    ],
    visibility = ["//visibility:public"],
)

ros2_cpp_library(
    name = "ros_adapter",
    srcs = ["src/ros_adapter.cpp"],
    hdrs = ["include/ros/ros_adapter.hpp"],
    includes = ["include"],
    deps = [
        "//core:core",
        "//viz:rerun_viz",
        ":ros_conversions",
        "//utils:stf",
    ],
    visibility = ["//visibility:public"],
)

# ros2_cpp_binary(
#     name = "rosbag_reader_main",
#     srcs = ["src/rosbag_reader_main.cpp"],
#     deps = [
#         ":rosbag_reader",
#     ],
#     visibility = ["//visibility:public"],
#     set_up_ament = True,
# )

# Test targets
cc_test(
    name = "conversions_test",
    srcs = ["test/conversions_test.cpp"],
    deps = [
        ":ros_conversions",
        "@com_google_googletest//:gtest_main",
    ],
)

# ros2_cpp_test(
#     name = "rosbag_reader_test",
#     srcs = ["test/rosbag_reader_test.cpp"],
#     deps = [
#         ":rosbag_reader",
#         "@com_google_googletest//:gtest_main",
#         "@ros2_rosbag2//:rosbag2_cpp",
#         "@ros2_rosbag2//:rosbag2_storage",
#     ],
#     data = [
#         "//ros/test/data:test_sequence.db3",
#     ],
# )

test_suite(
    name = "ros_tests",
    tests = [
        ":conversions_test",
        # ":rosbag_reader_test",
    ],
)
