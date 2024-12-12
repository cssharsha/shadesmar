load("@rules_cc//cc:defs.bzl", "cc_test")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_library")
load("@com_github_mvukov_rules_ros2//ros2:cc_defs.bzl", "ros2_cpp_test")

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
        ":ros_conversions",
        "@ros2_rosbag2//:rosbag2_cpp",          # ROS2 bag reading
        "@ros2_message_filters//:message_filters",
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

ros2_cpp_test(
    name = "rosbag_reader_test",
    srcs = ["test/rosbag_reader_test.cpp"],
    deps = [
        ":rosbag_reader",
        "@com_google_googletest//:gtest_main",
        "@ros2_rosbag2//:rosbag2_cpp",
        "@ros2_rosbag2//:rosbag2_storage",
    ],
    # data = [
    #     "@ros2_rosbag2//:rosbag2_cpp",  # The shared library
    #     "@ros2_rosbag2//:rosbag2_storage",  # The storage library
    #     "@ros2_rosbag2//:rosbag2_interfaces",
    # ],
    # env = {
    #     "AMENT_PREFIX_PATH": "external/ros2_rosbag2",
    #     "LD_LIBRARY_PATH": "external/ros2_rosbag2/lib",
    #     "ROSBAG2_STORAGE_DEFAULT_PLUGINS_PATH": "external/ros2_rosbag2/lib",
    # },
)

test_suite(
    name = "ros_tests",
    tests = [
        ":conversions_test",
        ":rosbag_reader_test",
    ],
)