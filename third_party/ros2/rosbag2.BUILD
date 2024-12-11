cc_library(
    name = "rosbag2_cpp",
    srcs = glob([
        "rosbag2_cpp/src/**/*.cpp",
        "rosbag2_cpp/src/**/*.hpp",
    ]),
    hdrs = glob(["rosbag2_cpp/include/**/*.hpp"]),
    includes = ["rosbag2_cpp/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_rclcpp//:rclcpp",
        "@ros2_rcpputils//:rcpputils",
        ":rosbag2_storage",
    ],
)

cc_library(
    name = "rosbag2_storage",
    srcs = glob([
        "rosbag2_storage/src/**/*.cpp",
        "rosbag2_storage/src/**/*.hpp",
    ]),
    hdrs = glob(["rosbag2_storage/include/**/*.hpp"]),
    includes = ["rosbag2_storage/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ros2_rcutils//:rcutils",
        "@ros2_rcpputils//:rcpputils",
    ],
)